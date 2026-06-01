#include "fruit_detect.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "jpeg_decoder.h"

static const char *TAG = "fruit_detect";

#define MAX_LABELS 512
#define MIN_BLOB_AREA 350
#define MIN_DIAMETER_PX 35
#define SMALL_MAX_DIAMETER_PX 45
#define MEDIUM_MAX_DIAMETER_PX 80

typedef struct {
    uint32_t sum_x;
    uint32_t sum_y;
    uint32_t count;
    uint16_t min_x;
    uint16_t max_x;
    uint16_t min_y;
    uint16_t max_y;
    uint32_t perimeter;
} component_stats_t;

static void *detect_malloc(size_t size)
{
    void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ptr) {
        ptr = malloc(size);
    }
    return ptr;
}

static void *detect_calloc(size_t count, size_t size)
{
    void *ptr = heap_caps_calloc(count, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ptr) {
        ptr = calloc(count, size);
    }
    return ptr;
}

static int max3(int a, int b, int c)
{
    int m = a > b ? a : b;
    return m > c ? m : c;
}

static int min3(int a, int b, int c)
{
    int m = a < b ? a : b;
    return m < c ? m : c;
}

static int rgb_hue_deg(uint8_t r, uint8_t g, uint8_t b)
{
    int ri = r;
    int gi = g;
    int bi = b;
    int maxc = max3(ri, gi, bi);
    int minc = min3(ri, gi, bi);
    int delta = maxc - minc;

    if (delta == 0) {
        return 0;
    }

    int hue;
    if (maxc == ri) {
        hue = 60 * (gi - bi) / delta;
        if (hue < 0) {
            hue += 360;
        }
    } else if (maxc == gi) {
        hue = 120 + 60 * (bi - ri) / delta;
    } else {
        hue = 240 + 60 * (ri - gi) / delta;
    }

    return hue;
}

static bool is_citrus_pixel(uint8_t r, uint8_t g, uint8_t b)
{
    int maxc = max3(r, g, b);
    int minc = min3(r, g, b);
    int delta = maxc - minc;

    if (maxc < 80 || delta < 35) {
        return false;
    }

    int saturation = delta * 255 / maxc;
    int hue = rgb_hue_deg(r, g, b);

    return saturation >= 70 &&
           hue >= 12 && hue <= 62 &&
           r >= 90 &&
           g >= 45 &&
           r >= b + 35 &&
           g >= b + 15 &&
           b * 100 <= maxc * 48 &&
           g * 100 >= r * 30 &&
           g * 100 <= r * 105;
}

static void open_mask_3x3(uint8_t *mask, uint8_t *scratch, uint16_t width, uint16_t height)
{
    memset(scratch, 0, (size_t)width * height);

    for (uint16_t y = 1; y + 1 < height; y++) {
        for (uint16_t x = 1; x + 1 < width; x++) {
            uint8_t count = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    count += mask[(size_t)(y + dy) * width + (x + dx)] ? 1 : 0;
                }
            }
            scratch[(size_t)y * width + x] = count >= 7 ? 1 : 0;
        }
    }

    memset(mask, 0, (size_t)width * height);

    for (uint16_t y = 1; y + 1 < height; y++) {
        for (uint16_t x = 1; x + 1 < width; x++) {
            size_t idx = (size_t)y * width + x;
            if (!scratch[idx]) {
                continue;
            }
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    mask[(size_t)(y + dy) * width + (x + dx)] = 1;
                }
            }
        }
    }
}

static uint16_t isqrt_u32(uint32_t value)
{
    uint32_t bit = 1UL << 30;
    uint32_t result = 0;

    while (bit > value) {
        bit >>= 2;
    }

    while (bit != 0) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }

    return (uint16_t)result;
}

static uint16_t clamp_u16(uint16_t value, uint16_t low, uint16_t high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static void uf_init(uint16_t *parent, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++) {
        parent[i] = i;
    }
}

static uint16_t uf_find(uint16_t *parent, uint16_t x)
{
    while (parent[x] != x) {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    return x;
}

static void uf_union(uint16_t *parent, uint16_t a, uint16_t b)
{
    uint16_t ra = uf_find(parent, a);
    uint16_t rb = uf_find(parent, b);

    if (ra == rb) {
        return;
    }

    if (ra < rb) {
        parent[rb] = ra;
    } else {
        parent[ra] = rb;
    }
}

const char *fruit_grade_label(fruit_grade_t grade)
{
    switch (grade) {
    case FRUIT_GRADE_LARGE:
        return "Large";
    case FRUIT_GRADE_MEDIUM:
        return "Medium";
    case FRUIT_GRADE_SMALL:
    default:
        return "Small";
    }
}

esp_err_t fruit_detect_init(void)
{
    ESP_LOGI(TAG, "Fruit detect ready");
    return ESP_OK;
}

esp_err_t fruit_detect_process(camera_fb_t *fb, fruit_detect_result_t *result)
{
    if (!fb || !fb->buf || fb->len == 0 || !result) {
        return ESP_ERR_INVALID_ARG;
    }
    if (fb->format != PIXFORMAT_JPEG) {
        ESP_LOGE(TAG, "Only JPEG frames are supported, format=%d", fb->format);
        return ESP_ERR_INVALID_ARG;
    }

    memset(result, 0, sizeof(*result));

    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = fb->buf,
        .indata_size = fb->len,
        .out_format = JPEG_IMAGE_FORMAT_RGB888,
        .out_scale = JPEG_IMAGE_SCALE_0,
    };
    esp_jpeg_image_output_t jpeg_out = {0};

    esp_err_t ret = esp_jpeg_get_image_info(&jpeg_cfg, &jpeg_out);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "jpeg info failed: 0x%x", ret);
        return ret;
    }

    uint16_t img_w = jpeg_out.width;
    uint16_t img_h = jpeg_out.height;
    size_t total_pixels = (size_t)img_w * img_h;

    result->image_width = img_w;
    result->image_height = img_h;

    uint8_t *rgb = detect_malloc(jpeg_out.output_len);
    if (!rgb) {
        ESP_LOGE(TAG, "No memory for RGB buffer, need %u bytes",
                 (unsigned int)jpeg_out.output_len);
        return ESP_ERR_NO_MEM;
    }

    jpeg_cfg.outbuf = rgb;
    jpeg_cfg.outbuf_size = jpeg_out.output_len;

    ret = esp_jpeg_decode(&jpeg_cfg, &jpeg_out);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "jpeg decode failed: 0x%x", ret);
        free(rgb);
        return ret;
    }

    uint8_t *mask = detect_malloc(total_pixels);
    if (!mask) {
        ESP_LOGE(TAG, "No memory for mask, need %u bytes", (unsigned int)total_pixels);
        free(rgb);
        return ESP_ERR_NO_MEM;
    }

    for (size_t i = 0; i < total_pixels; i++) {
        uint8_t r = rgb[i * 3];
        uint8_t g = rgb[i * 3 + 1];
        uint8_t b = rgb[i * 3 + 2];
        mask[i] = is_citrus_pixel(r, g, b) ? 1 : 0;
    }

    free(rgb);

    uint8_t *scratch = detect_malloc(total_pixels);
    if (!scratch) {
        ESP_LOGE(TAG, "No memory for mask scratch, need %u bytes", (unsigned int)total_pixels);
        free(mask);
        return ESP_ERR_NO_MEM;
    }
    open_mask_3x3(mask, scratch, img_w, img_h);
    free(scratch);

    uint16_t *labels = detect_calloc(total_pixels, sizeof(uint16_t));
    uint16_t *parent = detect_malloc(MAX_LABELS * sizeof(uint16_t));
    if (!labels || !parent) {
        ESP_LOGE(TAG, "No memory for connected components");
        free(mask);
        free(labels);
        free(parent);
        return ESP_ERR_NO_MEM;
    }

    uf_init(parent, MAX_LABELS);
    uint16_t next_label = 1;

    for (uint16_t y = 0; y < img_h; y++) {
        for (uint16_t x = 0; x < img_w; x++) {
            size_t idx = (size_t)y * img_w + x;
            if (!mask[idx]) {
                continue;
            }

            uint16_t up = y > 0 ? labels[(size_t)(y - 1) * img_w + x] : 0;
            uint16_t left = x > 0 ? labels[idx - 1] : 0;

            if (up == 0 && left == 0) {
                if (next_label < MAX_LABELS) {
                    labels[idx] = next_label++;
                }
            } else if (up != 0 && left != 0) {
                labels[idx] = up < left ? up : left;
                if (up != left) {
                    uf_union(parent, up, left);
                }
            } else {
                labels[idx] = up ? up : left;
            }
        }
    }

    free(mask);

    component_stats_t *stats = detect_calloc(MAX_LABELS, sizeof(component_stats_t));
    if (!stats) {
        ESP_LOGE(TAG, "No memory for component stats");
        free(labels);
        free(parent);
        return ESP_ERR_NO_MEM;
    }

    for (uint16_t i = 0; i < MAX_LABELS; i++) {
        stats[i].min_x = img_w;
        stats[i].min_y = img_h;
    }

    for (uint16_t y = 0; y < img_h; y++) {
        for (uint16_t x = 0; x < img_w; x++) {
            size_t idx = (size_t)y * img_w + x;
            uint16_t raw_label = labels[idx];
            if (raw_label == 0) {
                continue;
            }

            uint16_t root = uf_find(parent, raw_label);
            labels[idx] = root;
            component_stats_t *s = &stats[root];
            s->sum_x += x;
            s->sum_y += y;
            s->count++;

            if (x < s->min_x) {
                s->min_x = x;
            }
            if (x > s->max_x) {
                s->max_x = x;
            }
            if (y < s->min_y) {
                s->min_y = y;
            }
            if (y > s->max_y) {
                s->max_y = y;
            }
        }
    }

    for (uint16_t y = 0; y < img_h; y++) {
        for (uint16_t x = 0; x < img_w; x++) {
            size_t idx = (size_t)y * img_w + x;
            uint16_t root = labels[idx];
            if (root == 0) {
                continue;
            }

            bool edge = x == 0 || y == 0 || x + 1 == img_w || y + 1 == img_h ||
                        labels[idx - 1] != root ||
                        labels[idx + 1] != root ||
                        labels[(size_t)(y - 1) * img_w + x] != root ||
                        labels[(size_t)(y + 1) * img_w + x] != root;
            if (edge) {
                stats[root].perimeter++;
            }
        }
    }

    for (uint16_t i = 1; i < next_label && result->count < FRUIT_DETECT_MAX_FRUITS; i++) {
        if (uf_find(parent, i) != i) {
            continue;
        }

        component_stats_t *s = &stats[i];
        uint32_t dynamic_min_area = total_pixels / 220;
        if (dynamic_min_area < MIN_BLOB_AREA) {
            dynamic_min_area = MIN_BLOB_AREA;
        }
        if (s->count < dynamic_min_area || s->perimeter == 0) {
            continue;
        }

        uint16_t bbox_w = s->max_x - s->min_x + 1;
        uint16_t bbox_h = s->max_y - s->min_y + 1;
        uint16_t min_side = bbox_w < bbox_h ? bbox_w : bbox_h;
        uint16_t max_side = bbox_w > bbox_h ? bbox_w : bbox_h;
        uint32_t bbox_area = (uint32_t)bbox_w * bbox_h;
        uint32_t circularity100 =
            (uint32_t)((1256ULL * s->count) / ((uint64_t)s->perimeter * s->perimeter));

        if (min_side < 12 ||
            max_side > min_side * 2 ||
            s->count * 100 < bbox_area * 22 ||
            circularity100 < 32) {
            continue;
        }

        uint16_t area_diameter = isqrt_u32((uint32_t)((s->count * 400UL + 157UL) / 314UL));
        if (area_diameter < MIN_DIAMETER_PX || max_side < MIN_DIAMETER_PX) {
            continue;
        }
        uint16_t diameter = (area_diameter * 108U) / 100U;
        diameter = clamp_u16(diameter, MIN_DIAMETER_PX, max_side);

        fruit_grade_t grade = FRUIT_GRADE_SMALL;
        if (diameter > MEDIUM_MAX_DIAMETER_PX) {
            grade = FRUIT_GRADE_LARGE;
        } else if (diameter > SMALL_MAX_DIAMETER_PX) {
            grade = FRUIT_GRADE_MEDIUM;
        }

        fruit_info_t *f = &result->fruits[result->count++];
        f->center_x = (uint16_t)(s->sum_x / s->count);
        f->center_y = (uint16_t)(s->sum_y / s->count);
        f->diameter_px = diameter;
        uint16_t half = diameter / 2;
        f->bbox_x = f->center_x > half ? f->center_x - half : 0;
        f->bbox_y = f->center_y > half ? f->center_y - half : 0;
        if (f->bbox_x + diameter > img_w) {
            f->bbox_x = img_w > diameter ? img_w - diameter : 0;
        }
        if (f->bbox_y + diameter > img_h) {
            f->bbox_y = img_h > diameter ? img_h - diameter : 0;
        }
        f->bbox_w = diameter;
        f->bbox_h = diameter;
        f->area_px = s->count;
        f->size_grade = grade;
    }

    free(labels);
    free(stats);
    free(parent);

    for (uint8_t i = 1; i < result->count; i++) {
        fruit_info_t item = result->fruits[i];
        int j = i - 1;
        while (j >= 0 && result->fruits[j].area_px < item.area_px) {
            result->fruits[j + 1] = result->fruits[j];
            j--;
        }
        result->fruits[j + 1] = item;
    }

    ESP_LOGI(TAG, "Detected %u fruit(s) in %ux%u frame",
             result->count, img_w, img_h);
    return ESP_OK;
}
