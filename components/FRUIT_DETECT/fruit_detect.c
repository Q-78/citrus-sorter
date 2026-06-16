#include "fruit_detect.h"

#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "jpeg_decoder.h"

static const char *TAG = "fruit_detect";

#define MAX_LABELS 512
#define MIN_BLOB_AREA 350
#define MIN_DIAMETER_PX 35
#define LARGE_MIN_X_SPAN_PX 50

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

static void uf_init(uint16_t *parent, uint16_t count);
static uint16_t uf_find(uint16_t *parent, uint16_t x);
static void uf_union(uint16_t *parent, uint16_t a, uint16_t b);
static void stats_add_pixel(component_stats_t *s, uint16_t x, uint16_t y);

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

static uint8_t rgb_luma(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint8_t)(((int)r * 30 + (int)g * 59 + (int)b * 11) / 100);
}

static bool is_citrus_pixel(uint8_t r, uint8_t g, uint8_t b)
{
    int maxc = max3(r, g, b);
    int minc = min3(r, g, b);
    int delta = maxc - minc;

    if (maxc < 70 || delta < 25) {
        return false;
    }

    int saturation = delta * 255 / maxc;
    int hue = rgb_hue_deg(r, g, b);

    return saturation >= 45 &&
           hue >= 8 && hue <= 75 &&
           r >= 80 &&
           g >= 35 &&
           r >= b + 22 &&
           g >= b + 8 &&
           b * 100 <= maxc * 60 &&
           g * 100 >= r * 25 &&
           g * 100 <= r * 120;
}

static bool is_black_reference_pixel(uint8_t r, uint8_t g, uint8_t b)
{
    int maxc = max3(r, g, b);
    int minc = min3(r, g, b);
    int delta = maxc - minc;
    int luma = (int)r * 30 + (int)g * 59 + (int)b * 11;

    if (maxc == 0 || luma > 15000 || maxc > 175) {
        return false;
    }

    int saturation = delta * 255 / maxc;

    return saturation <= 155;
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

static float clamp_float(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static void update_board_bbox_from_corners(board_info_t *board,
                                           uint16_t img_w,
                                           uint16_t img_h)
{
    float max_x_limit = (float)(img_w - 1);
    float max_y_limit = (float)(img_h - 1);

    board->tl_x = clamp_float(board->tl_x, 0.0f, max_x_limit);
    board->tl_y = clamp_float(board->tl_y, 0.0f, max_y_limit);
    board->tr_x = clamp_float(board->tr_x, 0.0f, max_x_limit);
    board->tr_y = clamp_float(board->tr_y, 0.0f, max_y_limit);
    board->br_x = clamp_float(board->br_x, 0.0f, max_x_limit);
    board->br_y = clamp_float(board->br_y, 0.0f, max_y_limit);
    board->bl_x = clamp_float(board->bl_x, 0.0f, max_x_limit);
    board->bl_y = clamp_float(board->bl_y, 0.0f, max_y_limit);

    float min_xf = fminf(fminf(board->tl_x, board->tr_x),
                         fminf(board->br_x, board->bl_x));
    float max_xf = fmaxf(fmaxf(board->tl_x, board->tr_x),
                         fmaxf(board->br_x, board->bl_x));
    float min_yf = fminf(fminf(board->tl_y, board->tr_y),
                         fminf(board->br_y, board->bl_y));
    float max_yf = fmaxf(fmaxf(board->tl_y, board->tr_y),
                         fmaxf(board->br_y, board->bl_y));

    board->bbox_x = (uint16_t)clamp_u16((uint16_t)fmaxf(0.0f, floorf(min_xf)), 0, img_w - 1);
    board->bbox_y = (uint16_t)clamp_u16((uint16_t)fmaxf(0.0f, floorf(min_yf)), 0, img_h - 1);
    board->bbox_w = (uint16_t)fminf((float)(img_w - board->bbox_x),
                                    ceilf(max_xf) - (float)board->bbox_x + 1.0f);
    board->bbox_h = (uint16_t)fminf((float)(img_h - board->bbox_y),
                                    ceilf(max_yf) - (float)board->bbox_y + 1.0f);
    board->center_x = (uint16_t)(board->bbox_x + board->bbox_w / 2);
    board->center_y = (uint16_t)(board->bbox_y + board->bbox_h / 2);
}

static bool fit_vertical_line_x_at_y(const uint8_t *mask,
                                     uint16_t img_w,
                                     uint16_t img_h,
                                     uint16_t center_x,
                                     uint16_t y_top,
                                     uint16_t y_bottom,
                                     bool use_left_edge,
                                     float *x_at_top,
                                     float *x_at_bottom)
{
    if (!mask || !x_at_top || !x_at_bottom || y_bottom <= y_top) {
        return false;
    }

    int x0 = (int)center_x - 55;
    int x1 = (int)center_x + 55;
    if (x0 < 0) {
        x0 = 0;
    }
    if (x1 >= img_w) {
        x1 = img_w - 1;
    }

    float n = 0.0f;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_yy = 0.0f;
    float sum_xy = 0.0f;

    for (uint16_t y = y_top; y <= y_bottom && y < img_h; y++) {
        bool found = false;
        int edge_x = use_left_edge ? x1 : x0;
        for (int x = x0; x <= x1; x++) {
            if (!mask[(size_t)y * img_w + (uint16_t)x]) {
                continue;
            }
            if (!found ||
                (use_left_edge && x < edge_x) ||
                (!use_left_edge && x > edge_x)) {
                edge_x = x;
                found = true;
            }
        }
        if (found) {
            float xf = (float)edge_x;
            float yf = (float)y;
            n += 1.0f;
            sum_x += xf;
            sum_y += yf;
            sum_yy += yf * yf;
            sum_xy += xf * yf;
        }
    }

    if (n < (float)(y_bottom - y_top) * 0.25f) {
        return false;
    }

    float denom = n * sum_yy - sum_y * sum_y;
    if (fabsf(denom) < 0.0001f) {
        return false;
    }

    float a = (n * sum_xy - sum_y * sum_x) / denom;
    float b = (sum_x - a * sum_y) / n;
    *x_at_top = a * (float)y_top + b;
    *x_at_bottom = a * (float)y_bottom + b;
    return true;
}

static uint16_t vertical_window_votes(const uint8_t *mask,
                                      uint16_t img_w,
                                      uint16_t img_h,
                                      uint16_t center_x,
                                      uint16_t y_start,
                                      uint16_t y_end,
                                      uint16_t *min_y,
                                      uint16_t *max_y)
{
    int x0 = (int)center_x - 8;
    int x1 = (int)center_x + 8;
    if (x0 < 0) {
        x0 = 0;
    }
    if (x1 >= img_w) {
        x1 = img_w - 1;
    }

    uint16_t votes = 0;
    uint16_t first_y = img_h;
    uint16_t last_y = 0;
    for (uint16_t y = y_start; y < y_end && y < img_h; y++) {
        bool row_hit = false;
        for (int x = x0; x <= x1; x++) {
            if (mask[(size_t)y * img_w + (uint16_t)x]) {
                row_hit = true;
                votes++;
            }
        }
        if (row_hit) {
            if (y < first_y) {
                first_y = y;
            }
            last_y = y;
        }
    }

    if (min_y) {
        *min_y = first_y;
    }
    if (max_y) {
        *max_y = last_y;
    }
    return votes;
}

static bool find_horizontal_tick_y(const uint8_t *mask,
                                   uint16_t img_w,
                                   uint16_t img_h,
                                   uint16_t x0,
                                   uint16_t x1,
                                   uint16_t y0,
                                   uint16_t y1,
                                   uint16_t *best_y,
                                   uint16_t *best_score)
{
    if (!mask || !best_y || !best_score || x1 <= x0 || y1 <= y0) {
        return false;
    }
    if (x1 >= img_w) {
        x1 = img_w - 1;
    }
    if (y1 >= img_h) {
        y1 = img_h - 1;
    }

    uint16_t peak_y = y0;
    uint16_t peak_score = 0;
    for (uint16_t y = y0; y + 4 <= y1; y++) {
        uint16_t score = 0;
        for (uint16_t dy = 0; dy < 5; dy++) {
            for (uint16_t x = x0; x <= x1; x++) {
                if (mask[(size_t)(y + dy) * img_w + x]) {
                    score++;
                }
            }
        }
        if (score > peak_score) {
            peak_score = score;
            peak_y = y + 2;
        }
    }

    *best_score = peak_score;
    uint16_t min_score = (x1 - x0 + 1) / 3;
    if (min_score < 10) {
        min_score = 10;
    }
    if (peak_score < min_score) {
        return false;
    }

    *best_y = peak_y;
    return true;
}

typedef struct {
    bool found;
    uint16_t y;
    uint16_t corner_x;
    uint16_t far_x;
    uint16_t length;
} horizontal_tick_t;

static uint16_t reference_corner_vertical_support(const uint8_t *mask,
                                                  uint16_t img_w,
                                                  uint16_t img_h,
                                                  uint16_t corner_x,
                                                  uint16_t corner_y)
{
    int x_radius = img_w >= 480 ? 5 : 3;
    int y_radius = img_h / 8;
    if (y_radius < 16) {
        y_radius = 16;
    }
    if (y_radius > 70) {
        y_radius = 70;
    }

    int x0 = (int)corner_x - x_radius;
    int x1 = (int)corner_x + x_radius;
    int y0 = (int)corner_y - y_radius;
    int y1 = (int)corner_y + y_radius;
    if (x0 < 0) {
        x0 = 0;
    }
    if (x1 >= img_w) {
        x1 = img_w - 1;
    }
    if (y0 < 0) {
        y0 = 0;
    }
    if (y1 >= img_h) {
        y1 = img_h - 1;
    }

    uint16_t support = 0;
    for (int y = y0; y <= y1; y++) {
        bool row_hit = false;
        for (int x = x0; x <= x1; x++) {
            if (mask[(size_t)y * img_w + (uint16_t)x]) {
                row_hit = true;
                break;
            }
        }
        if (row_hit) {
            support++;
        }
    }

    return support;
}

static bool find_horizontal_tick(const uint8_t *mask,
                                 uint16_t img_w,
                                 uint16_t img_h,
                                 uint16_t x0,
                                 uint16_t x1,
                                 uint16_t y0,
                                 uint16_t y1,
                                 bool corner_on_left,
                                 horizontal_tick_t *tick)
{
    if (!mask || !tick || x1 <= x0 || y1 <= y0) {
        return false;
    }

    memset(tick, 0, sizeof(*tick));
    if (x1 >= img_w) {
        x1 = img_w - 1;
    }
    if (y1 >= img_h) {
        y1 = img_h - 1;
    }

    uint16_t min_len = img_w / 32;
    uint16_t max_len = img_w / 2;
    uint8_t max_gap = img_w >= 480 ? 7 : 5;
    uint16_t min_vertical_support = img_h / 22;
    if (min_len < 14) {
        min_len = 14;
    }
    if (min_vertical_support < 8) {
        min_vertical_support = 8;
    }

    for (uint16_t y = y0; y + 6 <= y1; y++) {
        bool in_run = false;
        uint16_t run_start = x0;
        uint16_t last_hit = x0;
        uint8_t gap = 0;

        for (uint16_t x = x0; x <= x1; x++) {
            bool hit = false;
            for (uint16_t dy = 0; dy < 7; dy++) {
                if (mask[(size_t)(y + dy) * img_w + x]) {
                    hit = true;
                    break;
                }
            }

            if (hit) {
                if (!in_run) {
                    in_run = true;
                    run_start = x;
                }
                last_hit = x;
                gap = 0;
            } else if (in_run) {
                gap++;
                if (gap > max_gap) {
                    uint16_t len = last_hit - run_start + 1;
                    uint16_t candidate_y = y + 3;
                    uint16_t candidate_corner_x = corner_on_left ? run_start : last_hit;
                    uint16_t support = reference_corner_vertical_support(mask, img_w, img_h,
                                                                         candidate_corner_x,
                                                                         candidate_y);
                    if (len >= min_len && len <= max_len &&
                        support >= min_vertical_support &&
                        len > tick->length) {
                        tick->found = true;
                        tick->y = candidate_y;
                        tick->corner_x = candidate_corner_x;
                        tick->far_x = corner_on_left ? last_hit : run_start;
                        tick->length = len;
                    }
                    in_run = false;
                    gap = 0;
                }
            }
        }

        if (in_run) {
            uint16_t len = last_hit - run_start + 1;
            uint16_t candidate_y = y + 3;
            uint16_t candidate_corner_x = corner_on_left ? run_start : last_hit;
            uint16_t support = reference_corner_vertical_support(mask, img_w, img_h,
                                                                 candidate_corner_x,
                                                                 candidate_y);
            if (len >= min_len && len <= max_len &&
                support >= min_vertical_support &&
                len > tick->length) {
                tick->found = true;
                tick->y = candidate_y;
                tick->corner_x = candidate_corner_x;
                tick->far_x = corner_on_left ? last_hit : run_start;
                tick->length = len;
            }
        }
    }

    return tick->found;
}

typedef struct {
    bool found;
    float a;
    float b;
    uint16_t points;
} line_y_fit_t;

typedef struct {
    bool found;
    float a;
    float b;
    uint16_t points;
} line_x_fit_t;

typedef struct {
    bool found;
    float x;
    float y;
    uint16_t score;
    uint16_t radius;
} reference_dot_t;

static uint16_t reference_line_support(const uint8_t *mask,
                                       uint16_t img_w,
                                       uint16_t img_h,
                                       int x,
                                       int y,
                                       int dx,
                                       int dy,
                                       uint16_t max_len,
                                       uint8_t band)
{
    uint16_t support = 0;
    for (uint16_t step = 0; step <= max_len; step++) {
        int cx = x + dx * (int)step;
        int cy = y + dy * (int)step;
        if (cx < 0 || cx >= img_w || cy < 0 || cy >= img_h) {
            break;
        }

        bool hit = false;
        for (int off = -(int)band; off <= (int)band; off++) {
            int sx = cx + (dy != 0 ? off : 0);
            int sy = cy + (dx != 0 ? off : 0);
            if (sx < 0 || sx >= img_w || sy < 0 || sy >= img_h) {
                continue;
            }
            if (mask[(size_t)sy * img_w + (uint16_t)sx]) {
                hit = true;
                break;
            }
        }
        if (hit) {
            support++;
        }
    }

    return support;
}

static bool find_reference_dot_near(const uint8_t *mask,
                                    uint16_t img_w,
                                    uint16_t img_h,
                                    float rough_x,
                                    float rough_y,
                                    reference_dot_t *dot)
{
    if (!mask || !dot) {
        return false;
    }

    memset(dot, 0, sizeof(*dot));

    int dot_r = img_w / 90;
    if (dot_r < 4) {
        dot_r = 4;
    }
    if (dot_r > 12) {
        dot_r = 12;
    }

    int search_rx = img_w / 16;
    int search_ry = img_h / 12;
    if (search_rx < 28) {
        search_rx = 28;
    }
    if (search_rx > 80) {
        search_rx = 80;
    }
    if (search_ry < 24) {
        search_ry = 24;
    }
    if (search_ry > 70) {
        search_ry = 70;
    }

    int center_x = (int)lroundf(rough_x);
    int center_y = (int)lroundf(rough_y);
    int x0 = center_x - search_rx;
    int x1 = center_x + search_rx;
    int y0 = center_y - search_ry;
    int y1 = center_y + search_ry;
    if (x0 < dot_r) {
        x0 = dot_r;
    }
    if (x1 >= (int)img_w - dot_r) {
        x1 = (int)img_w - dot_r - 1;
    }
    if (y0 < dot_r) {
        y0 = dot_r;
    }
    if (y1 >= (int)img_h - dot_r) {
        y1 = (int)img_h - dot_r - 1;
    }
    if (x1 <= x0 || y1 <= y0) {
        return false;
    }

    uint16_t disk_area = 0;
    for (int dy = -dot_r; dy <= dot_r; dy++) {
        for (int dx = -dot_r; dx <= dot_r; dx++) {
            if (dx * dx + dy * dy <= dot_r * dot_r) {
                disk_area++;
            }
        }
    }

    uint16_t best_count = 0;
    int best_x = center_x;
    int best_y = center_y;
    int best_dist2 = INT32_MAX;

    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            uint16_t count = 0;
            for (int dy = -dot_r; dy <= dot_r; dy++) {
                for (int dx = -dot_r; dx <= dot_r; dx++) {
                    if (dx * dx + dy * dy > dot_r * dot_r) {
                        continue;
                    }
                    if (mask[(size_t)(y + dy) * img_w + (uint16_t)(x + dx)]) {
                        count++;
                    }
                }
            }

            int ddx = x - center_x;
            int ddy = y - center_y;
            int dist2 = ddx * ddx + ddy * ddy;
            if (count > best_count ||
                (count == best_count && dist2 < best_dist2)) {
                best_count = count;
                best_x = x;
                best_y = y;
                best_dist2 = dist2;
            }
        }
    }

    uint16_t min_count = (uint16_t)((uint32_t)disk_area * 38U / 100U);
    if (min_count < 16) {
        min_count = 16;
    }
    if (best_count < min_count) {
        return false;
    }

    dot->found = true;
    dot->x = (float)best_x;
    dot->y = (float)best_y;
    dot->score = best_count;
    dot->radius = (uint16_t)dot_r;
    return true;
}

static bool fit_reference_horizontal_line(const uint8_t *mask,
                                          uint16_t img_w,
                                          uint16_t img_h,
                                          const horizontal_tick_t *tick,
                                          int dir_x,
                                          line_y_fit_t *line)
{
    if (!mask || !tick || !line || !tick->found || dir_x == 0) {
        return false;
    }

    memset(line, 0, sizeof(*line));

    int x0 = tick->corner_x < tick->far_x ? tick->corner_x : tick->far_x;
    int x1 = tick->corner_x > tick->far_x ? tick->corner_x : tick->far_x;
    int y_radius = img_h / 80;
    if (y_radius < 8) {
        y_radius = 8;
    }
    if (y_radius > 18) {
        y_radius = 18;
    }
    x0 -= 4;
    x1 += 4;
    int y0 = (int)tick->y - y_radius;
    int y1 = (int)tick->y + y_radius;
    if (x0 < 0) {
        x0 = 0;
    }
    if (x1 >= img_w) {
        x1 = img_w - 1;
    }
    if (y0 < 0) {
        y0 = 0;
    }
    if (y1 >= img_h) {
        y1 = img_h - 1;
    }

    uint16_t support_len = img_w / 50;
    if (support_len < 10) {
        support_len = 10;
    }
    if (support_len > 28) {
        support_len = 28;
    }
    uint16_t min_support = support_len / 2;
    if (min_support < 6) {
        min_support = 6;
    }

    float n = 0.0f;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;

    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            if (!mask[(size_t)y * img_w + (uint16_t)x]) {
                continue;
            }
            uint16_t support_a = reference_line_support(mask, img_w, img_h,
                                                        x, y, dir_x, 0,
                                                        support_len, 2);
            uint16_t support_b = reference_line_support(mask, img_w, img_h,
                                                        x, y, -dir_x, 0,
                                                        support_len, 2);
            uint16_t support = support_a > support_b ? support_a : support_b;
            if (support < min_support) {
                continue;
            }

            float xf = (float)x;
            float yf = (float)y;
            n += 1.0f;
            sum_x += xf;
            sum_y += yf;
            sum_xx += xf * xf;
            sum_xy += xf * yf;
        }
    }

    float min_points = (float)tick->length * 0.30f;
    if (min_points < 14.0f) {
        min_points = 14.0f;
    }
    if (n < min_points) {
        return false;
    }

    float denom = n * sum_xx - sum_x * sum_x;
    if (fabsf(denom) < 0.0001f) {
        return false;
    }

    line->a = (n * sum_xy - sum_x * sum_y) / denom;
    line->b = (sum_y - line->a * sum_x) / n;
    line->points = (uint16_t)n;
    line->found = true;
    return true;
}

static bool fit_reference_horizontal_line_between(const uint8_t *mask,
                                                  uint16_t img_w,
                                                  uint16_t img_h,
                                                  uint16_t x0,
                                                  uint16_t x1,
                                                  uint16_t y_a,
                                                  uint16_t y_b,
                                                  line_y_fit_t *line)
{
    if (!mask || !line || x1 <= x0) {
        return false;
    }

    memset(line, 0, sizeof(*line));

    int y_radius = img_h / 45;
    if (y_radius < 12) {
        y_radius = 12;
    }
    if (y_radius > 32) {
        y_radius = 32;
    }

    int y0 = (int)(y_a < y_b ? y_a : y_b) - y_radius;
    int y1 = (int)(y_a > y_b ? y_a : y_b) + y_radius;
    if (x1 >= img_w) {
        x1 = img_w - 1;
    }
    if (y0 < 0) {
        y0 = 0;
    }
    if (y1 >= img_h) {
        y1 = img_h - 1;
    }

    uint16_t support_len = img_w / 38;
    if (support_len < 14) {
        support_len = 14;
    }
    if (support_len > 42) {
        support_len = 42;
    }
    uint16_t min_support = support_len / 2;
    if (min_support < 8) {
        min_support = 8;
    }

    float n = 0.0f;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;

    for (int y = y0; y <= y1; y++) {
        for (uint16_t x = x0; x <= x1; x++) {
            if (!mask[(size_t)y * img_w + x]) {
                continue;
            }
            uint16_t support_right = reference_line_support(mask, img_w, img_h,
                                                            x, y, 1, 0,
                                                            support_len, 2);
            uint16_t support_left = reference_line_support(mask, img_w, img_h,
                                                           x, y, -1, 0,
                                                           support_len, 2);
            uint16_t support = support_right > support_left ? support_right : support_left;
            if (support < min_support) {
                continue;
            }

            float xf = (float)x;
            float yf = (float)y;
            n += 1.0f;
            sum_x += xf;
            sum_y += yf;
            sum_xx += xf * xf;
            sum_xy += xf * yf;
        }
    }

    float min_points = (float)(x1 - x0 + 1) * 0.10f;
    if (min_points < 18.0f) {
        min_points = 18.0f;
    }
    if (n < min_points) {
        return false;
    }

    float denom = n * sum_xx - sum_x * sum_x;
    if (fabsf(denom) < 0.0001f) {
        return false;
    }

    line->a = (n * sum_xy - sum_x * sum_y) / denom;
    line->b = (sum_y - line->a * sum_x) / n;
    line->points = (uint16_t)n;
    line->found = true;
    return true;
}

static bool fit_reference_vertical_line(const uint8_t *mask,
                                        uint16_t img_w,
                                        uint16_t img_h,
                                        uint16_t rough_x,
                                        uint16_t y_top,
                                        uint16_t y_bottom,
                                        line_x_fit_t *line)
{
    if (!mask || !line || y_bottom <= y_top) {
        return false;
    }

    memset(line, 0, sizeof(*line));

    int x_radius = img_w / 12;
    if (x_radius < 32) {
        x_radius = 32;
    }
    if (x_radius > 90) {
        x_radius = 90;
    }
    int x0 = (int)rough_x - x_radius;
    int x1 = (int)rough_x + x_radius;
    int y0 = (int)y_top - 4;
    int y1 = (int)y_bottom + 4;
    if (x0 < 0) {
        x0 = 0;
    }
    if (x1 >= img_w) {
        x1 = img_w - 1;
    }
    if (y0 < 0) {
        y0 = 0;
    }
    if (y1 >= img_h) {
        y1 = img_h - 1;
    }

    uint16_t support_len = img_h / 45;
    if (support_len < 10) {
        support_len = 10;
    }
    if (support_len > 30) {
        support_len = 30;
    }
    uint16_t min_support = support_len / 2;
    if (min_support < 6) {
        min_support = 6;
    }

    int best_x = rough_x;
    uint16_t best_score = 0;
    for (int x = x0; x <= x1; x++) {
        uint16_t score = 0;
        for (int y = y0; y <= y1; y++) {
            if (!mask[(size_t)y * img_w + (uint16_t)x]) {
                continue;
            }
            uint16_t support_down = reference_line_support(mask, img_w, img_h,
                                                           x, y, 0, 1,
                                                           support_len, 2);
            uint16_t support_up = reference_line_support(mask, img_w, img_h,
                                                         x, y, 0, -1,
                                                         support_len, 2);
            uint16_t support = support_down > support_up ? support_down : support_up;
            if (support >= min_support) {
                score++;
            }
        }

        int best_distance = best_x > rough_x ? best_x - rough_x : rough_x - best_x;
        int candidate_distance = x > rough_x ? x - rough_x : rough_x - x;
        if (score > best_score ||
            (score == best_score && candidate_distance < best_distance)) {
            best_score = score;
            best_x = x;
        }
    }

    uint16_t min_column_score = (y_bottom - y_top) / 6;
    if (min_column_score < 10) {
        min_column_score = 10;
    }
    if (best_score < min_column_score) {
        return false;
    }

    int fit_radius = img_w / 16;
    if (fit_radius < 24) {
        fit_radius = 24;
    }
    if (fit_radius > 45) {
        fit_radius = 45;
    }
    x0 = best_x - fit_radius;
    x1 = best_x + fit_radius;
    if (x0 < 0) {
        x0 = 0;
    }
    if (x1 >= img_w) {
        x1 = img_w - 1;
    }

    float n = 0.0f;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_yy = 0.0f;
    float sum_xy = 0.0f;

    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            if (!mask[(size_t)y * img_w + (uint16_t)x]) {
                continue;
            }
            uint16_t support_down = reference_line_support(mask, img_w, img_h,
                                                           x, y, 0, 1,
                                                           support_len, 2);
            uint16_t support_up = reference_line_support(mask, img_w, img_h,
                                                         x, y, 0, -1,
                                                         support_len, 2);
            uint16_t support = support_down > support_up ? support_down : support_up;
            if (support < min_support) {
                continue;
            }

            float xf = (float)x;
            float yf = (float)y;
            n += 1.0f;
            sum_x += xf;
            sum_y += yf;
            sum_yy += yf * yf;
            sum_xy += xf * yf;
        }
    }

    float min_points = (float)(y_bottom - y_top) * 0.30f;
    if (min_points < 20.0f) {
        min_points = 20.0f;
    }
    if (n < min_points) {
        return false;
    }

    float denom = n * sum_yy - sum_y * sum_y;
    if (fabsf(denom) < 0.0001f) {
        return false;
    }

    line->a = (n * sum_xy - sum_y * sum_x) / denom;
    line->b = (sum_x - line->a * sum_y) / n;
    line->points = (uint16_t)n;
    line->found = true;
    return true;
}

static bool intersect_reference_lines(const line_y_fit_t *horizontal,
                                      const line_x_fit_t *vertical,
                                      float *x,
                                      float *y)
{
    if (!horizontal || !vertical || !horizontal->found || !vertical->found ||
        !x || !y) {
        return false;
    }

    float denom = 1.0f - horizontal->a * vertical->a;
    if (fabsf(denom) < 0.0001f) {
        return false;
    }

    *y = (horizontal->a * vertical->b + horizontal->b) / denom;
    *x = vertical->a * (*y) + vertical->b;
    return true;
}

static bool reference_point_reasonable(float x,
                                       float y,
                                       uint16_t img_w,
                                       uint16_t img_h)
{
    float x_margin = (float)img_w * 0.08f;
    float y_margin = (float)img_h * 0.08f;
    return x >= -x_margin &&
           x <= (float)img_w + x_margin &&
           y >= -y_margin &&
           y <= (float)img_h + y_margin;
}

static void blend_reference_dot(const reference_dot_t *dot,
                                float *x,
                                float *y)
{
    if (!dot || !dot->found || !x || !y) {
        return;
    }

    *x = dot->x * 0.82f + *x * 0.18f;
    *y = dot->y * 0.82f + *y * 0.18f;
}

static bool detect_reference_from_ticks(const uint8_t *mask,
                                        uint16_t img_w,
                                        uint16_t img_h,
                                        board_info_t *board)
{
    horizontal_tick_t lt;
    horizontal_tick_t lb;
    horizontal_tick_t rt;
    horizontal_tick_t rb;
    uint16_t left_x0 = img_w / 100;
    uint16_t left_x1 = (uint16_t)((uint32_t)img_w * 45U / 100U);
    uint16_t right_x0 = (uint16_t)((uint32_t)img_w * 55U / 100U);
    uint16_t right_x1 = img_w - img_w / 100 - 1;
    uint16_t top_y0 = img_h / 8;
    uint16_t top_y1 = img_h / 2;
    uint16_t bottom_y0 = img_h / 2;
    uint16_t bottom_y1 = img_h - img_h / 12;

    if (!find_horizontal_tick(mask, img_w, img_h, left_x0, left_x1,
                              top_y0, top_y1, true, &lt) ||
        !find_horizontal_tick(mask, img_w, img_h, left_x0, left_x1,
                              bottom_y0, bottom_y1, true, &lb) ||
        !find_horizontal_tick(mask, img_w, img_h, right_x0, right_x1,
                              top_y0, top_y1, false, &rt) ||
        !find_horizontal_tick(mask, img_w, img_h, right_x0, right_x1,
                              bottom_y0, bottom_y1, false, &rb)) {
        return false;
    }

    if (rt.corner_x <= lt.corner_x + img_w / 3 ||
        rb.corner_x <= lb.corner_x + img_w / 3 ||
        lb.y <= lt.y + img_h / 5 ||
        rb.y <= rt.y + img_h / 5) {
        return false;
    }

    float left_top_x = (float)lt.corner_x;
    float left_bottom_x = (float)lb.corner_x;
    float right_top_x = (float)rt.corner_x;
    float right_bottom_x = (float)rb.corner_x;
    uint16_t left_center_x = (uint16_t)(((uint32_t)lt.corner_x + lb.corner_x) / 2U);
    uint16_t right_center_x = (uint16_t)(((uint32_t)rt.corner_x + rb.corner_x) / 2U);

    line_x_fit_t left_vertical;
    line_x_fit_t right_vertical;
    line_y_fit_t lt_horizontal;
    line_y_fit_t lb_horizontal;
    line_y_fit_t rt_horizontal;
    line_y_fit_t rb_horizontal;
    bool left_vertical_ok = fit_reference_vertical_line(mask, img_w, img_h,
                                                        left_center_x, lt.y, lb.y,
                                                        &left_vertical);
    bool right_vertical_ok = fit_reference_vertical_line(mask, img_w, img_h,
                                                         right_center_x, rt.y, rb.y,
                                                         &right_vertical);
    bool lt_horizontal_ok = fit_reference_horizontal_line(mask, img_w, img_h,
                                                          &lt, 1, &lt_horizontal);
    bool lb_horizontal_ok = fit_reference_horizontal_line(mask, img_w, img_h,
                                                          &lb, 1, &lb_horizontal);
    bool rt_horizontal_ok = fit_reference_horizontal_line(mask, img_w, img_h,
                                                          &rt, -1, &rt_horizontal);
    bool rb_horizontal_ok = fit_reference_horizontal_line(mask, img_w, img_h,
                                                          &rb, -1, &rb_horizontal);

    if (left_vertical_ok) {
        left_top_x = left_vertical.a * (float)lt.y + left_vertical.b;
        left_bottom_x = left_vertical.a * (float)lb.y + left_vertical.b;
    } else {
        fit_vertical_line_x_at_y(mask, img_w, img_h, left_center_x, lt.y, lb.y, true,
                                 &left_top_x, &left_bottom_x);
    }

    if (right_vertical_ok) {
        right_top_x = right_vertical.a * (float)rt.y + right_vertical.b;
        right_bottom_x = right_vertical.a * (float)rb.y + right_vertical.b;
    } else {
        fit_vertical_line_x_at_y(mask, img_w, img_h, right_center_x, rt.y, rb.y, false,
                                 &right_top_x, &right_bottom_x);
    }

    float tl_x = left_top_x;
    float tl_y = (float)lt.y;
    float tr_x = right_top_x;
    float tr_y = (float)rt.y;
    float br_x = right_bottom_x;
    float br_y = (float)rb.y;
    float bl_x = left_bottom_x;
    float bl_y = (float)lb.y;

    float ix;
    float iy;
    if (left_vertical_ok && lt_horizontal_ok &&
        intersect_reference_lines(&lt_horizontal, &left_vertical, &ix, &iy) &&
        reference_point_reasonable(ix, iy, img_w, img_h)) {
        tl_x = ix;
        tl_y = iy;
    }
    if (right_vertical_ok && rt_horizontal_ok &&
        intersect_reference_lines(&rt_horizontal, &right_vertical, &ix, &iy) &&
        reference_point_reasonable(ix, iy, img_w, img_h)) {
        tr_x = ix;
        tr_y = iy;
    }
    if (right_vertical_ok && rb_horizontal_ok &&
        intersect_reference_lines(&rb_horizontal, &right_vertical, &ix, &iy) &&
        reference_point_reasonable(ix, iy, img_w, img_h)) {
        br_x = ix;
        br_y = iy;
    }
    if (left_vertical_ok && lb_horizontal_ok &&
        intersect_reference_lines(&lb_horizontal, &left_vertical, &ix, &iy) &&
        reference_point_reasonable(ix, iy, img_w, img_h)) {
        bl_x = ix;
        bl_y = iy;
    }

    reference_dot_t tl_dot;
    reference_dot_t tr_dot;
    reference_dot_t br_dot;
    reference_dot_t bl_dot;
    find_reference_dot_near(mask, img_w, img_h, tl_x, tl_y, &tl_dot);
    find_reference_dot_near(mask, img_w, img_h, tr_x, tr_y, &tr_dot);
    find_reference_dot_near(mask, img_w, img_h, br_x, br_y, &br_dot);
    find_reference_dot_near(mask, img_w, img_h, bl_x, bl_y, &bl_dot);

    blend_reference_dot(&tl_dot, &tl_x, &tl_y);
    blend_reference_dot(&tr_dot, &tr_x, &tr_y);
    blend_reference_dot(&br_dot, &br_x, &br_y);
    blend_reference_dot(&bl_dot, &bl_x, &bl_y);

    board->found = true;
    board->tl_x = tl_x;
    board->tl_y = tl_y;
    board->tr_x = tr_x;
    board->tr_y = tr_y;
    board->br_x = br_x;
    board->br_y = br_y;
    board->bl_x = bl_x;
    board->bl_y = bl_y;
    board->area_px = lt.length + lb.length + rt.length + rb.length;
    update_board_bbox_from_corners(board, img_w, img_h);
    return true;
}

static float distance_f(float ax, float ay, float bx, float by)
{
    float dx = bx - ax;
    float dy = by - ay;
    return sqrtf(dx * dx + dy * dy);
}

static bool solve_8x8(float a[8][9], float out[8])
{
    for (uint8_t col = 0; col < 8; col++) {
        uint8_t pivot = col;
        float best = fabsf(a[col][col]);
        for (uint8_t row = col + 1; row < 8; row++) {
            float value = fabsf(a[row][col]);
            if (value > best) {
                best = value;
                pivot = row;
            }
        }

        if (best < 0.000001f) {
            return false;
        }

        if (pivot != col) {
            for (uint8_t k = col; k < 9; k++) {
                float tmp = a[col][k];
                a[col][k] = a[pivot][k];
                a[pivot][k] = tmp;
            }
        }

        float div = a[col][col];
        for (uint8_t k = col; k < 9; k++) {
            a[col][k] /= div;
        }

        for (uint8_t row = 0; row < 8; row++) {
            if (row == col) {
                continue;
            }
            float factor = a[row][col];
            if (fabsf(factor) < 0.000001f) {
                continue;
            }
            for (uint8_t k = col; k < 9; k++) {
                a[row][k] -= factor * a[col][k];
            }
        }
    }

    for (uint8_t i = 0; i < 8; i++) {
        out[i] = a[i][8];
    }
    return true;
}

static bool solve_image_to_unit_homography(const board_info_t *board, float h[9])
{
    const float src_x[4] = {
        board->tl_x, board->tr_x, board->br_x, board->bl_x,
    };
    const float src_y[4] = {
        board->tl_y, board->tr_y, board->br_y, board->bl_y,
    };
    const float dst_u[4] = {0.0f, 1.0f, 1.0f, 0.0f};
    const float dst_v[4] = {0.0f, 0.0f, 1.0f, 1.0f};
    float a[8][9] = {0};

    for (uint8_t i = 0; i < 4; i++) {
        float x = src_x[i];
        float y = src_y[i];
        float u = dst_u[i];
        float v = dst_v[i];
        uint8_t r = i * 2;

        a[r][0] = x;
        a[r][1] = y;
        a[r][2] = 1.0f;
        a[r][6] = -u * x;
        a[r][7] = -u * y;
        a[r][8] = u;

        a[r + 1][3] = x;
        a[r + 1][4] = y;
        a[r + 1][5] = 1.0f;
        a[r + 1][6] = -v * x;
        a[r + 1][7] = -v * y;
        a[r + 1][8] = v;
    }

    float solved[8];
    if (!solve_8x8(a, solved)) {
        return false;
    }

    for (uint8_t i = 0; i < 8; i++) {
        h[i] = solved[i];
    }
    h[8] = 1.0f;
    return true;
}

bool fruit_detect_board_relative_coord(const board_info_t *board,
                                       uint16_t pixel_x,
                                       uint16_t pixel_y,
                                       float *relative_x,
                                       float *relative_y)
{
    if (!board || !board->found || !relative_x || !relative_y) {
        return false;
    }

    float h[9];
    if (!solve_image_to_unit_homography(board, h)) {
        return false;
    }

    float x = (float)pixel_x;
    float y = (float)pixel_y;
    float denom = h[6] * x + h[7] * y + h[8];
    if (fabsf(denom) < 0.000001f) {
        return false;
    }

    float u = (h[0] * x + h[1] * y + h[2]) / denom;
    float v = (h[3] * x + h[4] * y + h[5]) / denom;
    float width = (distance_f(board->tl_x, board->tl_y, board->tr_x, board->tr_y) +
                   distance_f(board->bl_x, board->bl_y, board->br_x, board->br_y)) * 0.5f;
    float height = (distance_f(board->tl_x, board->tl_y, board->bl_x, board->bl_y) +
                    distance_f(board->tr_x, board->tr_y, board->br_x, board->br_y)) * 0.5f;

    *relative_x = u * width;
    *relative_y = v * height;
    return true;
}

static void detect_board_from_mask(uint8_t *mask,
                                   uint16_t img_w,
                                   uint16_t img_h,
                                   board_info_t *board)
{
    memset(board, 0, sizeof(*board));

    size_t total_pixels = (size_t)img_w * img_h;
    if (detect_reference_from_ticks(mask, img_w, img_h, board)) {
        return;
    }

    uint16_t *col_counts = detect_calloc(img_w, sizeof(uint16_t));
    if (col_counts) {
        uint16_t y_start = img_h / 8;
        uint16_t y_end = img_h - img_h / 6;
        uint16_t x_start = img_w / 20;
        uint16_t x_end = img_w - img_w / 20;
        uint16_t left_x_end = (uint16_t)((uint32_t)img_w * 38U / 100U);
        uint16_t right_x_start = (uint16_t)((uint32_t)img_w * 62U / 100U);
        if (left_x_end <= x_start) {
            left_x_end = img_w / 2;
        }
        if (right_x_start + 5 >= x_end) {
            right_x_start = img_w / 2;
        }
        for (uint16_t y = 0; y < img_h; y++) {
            for (uint16_t x = 0; x < img_w; x++) {
                if (mask[(size_t)y * img_w + x]) {
                    if (y >= y_start && y < y_end) {
                        col_counts[x]++;
                    }
                }
            }
        }

        uint16_t min_vertical_votes = img_h / 8;
        uint16_t tick_len = img_w / 7;
        if (tick_len < 28) {
            tick_len = 28;
        }
        if (tick_len > 96) {
            tick_len = 96;
        }
        uint16_t left_x = 0;
        uint16_t right_x = 0;
        uint16_t left_score = 0;
        uint16_t right_score = 0;
        uint32_t left_best_score = 0;
        uint32_t right_best_score = 0;
        uint16_t left_min_y = 0;
        uint16_t left_max_y = 0;
        uint16_t right_min_y = 0;
        uint16_t right_max_y = 0;
        uint16_t left_top_y = 0;
        uint16_t right_top_y = 0;
        uint16_t left_bottom_y = 0;
        uint16_t right_bottom_y = 0;

        for (uint16_t x = x_start; x < left_x_end; x++) {
            uint16_t score = 0;
            for (uint16_t dx = 0; dx < 5 && x + dx < img_w; dx++) {
                score += col_counts[x + dx];
            }
            uint16_t candidate_min_y = 0;
            uint16_t candidate_max_y = 0;
            uint16_t votes = vertical_window_votes(mask, img_w, img_h, x + 2,
                                                   y_start, y_end,
                                                   &candidate_min_y,
                                                   &candidate_max_y);
            if (score >= min_vertical_votes &&
                votes >= min_vertical_votes &&
                candidate_max_y > candidate_min_y + img_h / 4) {
                uint16_t tick_x1 = x + 2 + tick_len < img_w ? x + 2 + tick_len : img_w - 1;
                uint16_t top_y = candidate_min_y;
                uint16_t bottom_y = candidate_max_y;
                uint16_t top_score = 0;
                uint16_t bottom_score = 0;
                bool top_ok = find_horizontal_tick_y(mask, img_w, img_h,
                                                     x + 2, tick_x1,
                                                     y_start, img_h / 2,
                                                     &top_y, &top_score);
                bool bottom_ok = find_horizontal_tick_y(mask, img_w, img_h,
                                                        x + 2, tick_x1,
                                                        img_h / 2, y_end,
                                                        &bottom_y, &bottom_score);
                if (!top_ok || !bottom_ok) {
                    continue;
                }
                uint32_t candidate_score = (uint32_t)score + votes +
                                           (uint32_t)top_score * 4U +
                                           (uint32_t)bottom_score * 4U;
                if (candidate_score > left_best_score ||
                    (candidate_score == left_best_score && x + 2 < left_x)) {
                    left_best_score = candidate_score;
                    left_score = score;
                    left_x = x + 2;
                    left_min_y = candidate_min_y;
                    left_max_y = candidate_max_y;
                    left_top_y = top_y;
                    left_bottom_y = bottom_y;
                }
            }
        }

        for (uint16_t x = right_x_start; x + 5 < x_end; x++) {
            uint16_t score = 0;
            for (uint16_t dx = 0; dx < 5; dx++) {
                score += col_counts[x + dx];
            }
            uint16_t candidate_min_y = 0;
            uint16_t candidate_max_y = 0;
            uint16_t votes = vertical_window_votes(mask, img_w, img_h, x + 2,
                                                   y_start, y_end,
                                                   &candidate_min_y,
                                                   &candidate_max_y);
            if (score >= min_vertical_votes &&
                votes >= min_vertical_votes &&
                candidate_max_y > candidate_min_y + img_h / 4) {
                uint16_t tick_x0 = x + 2 > tick_len ? x + 2 - tick_len : 0;
                uint16_t top_y = candidate_min_y;
                uint16_t bottom_y = candidate_max_y;
                uint16_t top_score = 0;
                uint16_t bottom_score = 0;
                bool top_ok = find_horizontal_tick_y(mask, img_w, img_h,
                                                     tick_x0, x + 2,
                                                     y_start, img_h / 2,
                                                     &top_y, &top_score);
                bool bottom_ok = find_horizontal_tick_y(mask, img_w, img_h,
                                                        tick_x0, x + 2,
                                                        img_h / 2, y_end,
                                                        &bottom_y, &bottom_score);
                if (!top_ok || !bottom_ok) {
                    continue;
                }
                uint32_t candidate_score = (uint32_t)score + votes +
                                           (uint32_t)top_score * 4U +
                                           (uint32_t)bottom_score * 4U;
                if (candidate_score > right_best_score ||
                    (candidate_score == right_best_score && x + 2 > right_x)) {
                    right_best_score = candidate_score;
                    right_score = score;
                    right_x = x + 2;
                    right_min_y = candidate_min_y;
                    right_max_y = candidate_max_y;
                    right_top_y = top_y;
                    right_bottom_y = bottom_y;
                }
            }
        }

        uint16_t top_y = left_min_y < right_min_y ? left_min_y : right_min_y;
        uint16_t bottom_y = left_max_y > right_max_y ? left_max_y : right_max_y;

        if (left_score >= min_vertical_votes &&
            right_score >= min_vertical_votes &&
            right_x > left_x + img_w / 4 &&
            bottom_y > top_y + img_h / 5) {
            float left_top_x = (float)left_x;
            float left_bottom_x = (float)left_x;
            float right_top_x = (float)right_x;
            float right_bottom_x = (float)right_x;
            line_x_fit_t left_vertical;
            line_x_fit_t right_vertical;
            line_y_fit_t top_horizontal;
            line_y_fit_t bottom_horizontal;
            bool left_vertical_ok = fit_reference_vertical_line(mask, img_w, img_h,
                                                                left_x, left_top_y, left_bottom_y,
                                                                &left_vertical);
            bool right_vertical_ok = fit_reference_vertical_line(mask, img_w, img_h,
                                                                 right_x, right_top_y, right_bottom_y,
                                                                 &right_vertical);
            bool top_horizontal_ok = fit_reference_horizontal_line_between(mask, img_w, img_h,
                                                                           left_x, right_x,
                                                                           left_top_y, right_top_y,
                                                                           &top_horizontal);
            bool bottom_horizontal_ok = fit_reference_horizontal_line_between(mask, img_w, img_h,
                                                                              left_x, right_x,
                                                                              left_bottom_y, right_bottom_y,
                                                                              &bottom_horizontal);
            if (left_vertical_ok) {
                left_top_x = left_vertical.a * (float)left_top_y + left_vertical.b;
                left_bottom_x = left_vertical.a * (float)left_bottom_y + left_vertical.b;
            } else {
                fit_vertical_line_x_at_y(mask, img_w, img_h, left_x, left_top_y, left_bottom_y, true,
                                         &left_top_x, &left_bottom_x);
            }
            if (right_vertical_ok) {
                right_top_x = right_vertical.a * (float)right_top_y + right_vertical.b;
                right_bottom_x = right_vertical.a * (float)right_bottom_y + right_vertical.b;
            } else {
                fit_vertical_line_x_at_y(mask, img_w, img_h, right_x, right_top_y, right_bottom_y, false,
                                         &right_top_x, &right_bottom_x);
            }

            board->found = true;
            board->tl_x = left_top_x;
            board->tl_y = (float)left_top_y;
            board->tr_x = right_top_x;
            board->tr_y = (float)right_top_y;
            board->br_x = right_bottom_x;
            board->br_y = (float)right_bottom_y;
            board->bl_x = left_bottom_x;
            board->bl_y = (float)left_bottom_y;
            board->area_px = left_score + right_score;
            float ix;
            float iy;
            if (left_vertical_ok && top_horizontal_ok &&
                intersect_reference_lines(&top_horizontal, &left_vertical, &ix, &iy) &&
                reference_point_reasonable(ix, iy, img_w, img_h)) {
                board->tl_x = ix;
                board->tl_y = iy;
            }
            if (right_vertical_ok && top_horizontal_ok &&
                intersect_reference_lines(&top_horizontal, &right_vertical, &ix, &iy) &&
                reference_point_reasonable(ix, iy, img_w, img_h)) {
                board->tr_x = ix;
                board->tr_y = iy;
            }
            if (right_vertical_ok && bottom_horizontal_ok &&
                intersect_reference_lines(&bottom_horizontal, &right_vertical, &ix, &iy) &&
                reference_point_reasonable(ix, iy, img_w, img_h)) {
                board->br_x = ix;
                board->br_y = iy;
            }
            if (left_vertical_ok && bottom_horizontal_ok &&
                intersect_reference_lines(&bottom_horizontal, &left_vertical, &ix, &iy) &&
                reference_point_reasonable(ix, iy, img_w, img_h)) {
                board->bl_x = ix;
                board->bl_y = iy;
            }
            update_board_bbox_from_corners(board, img_w, img_h);
        }
    }
    free(col_counts);
    if (board->found) {
        return;
    }

    uint16_t *labels = detect_calloc(total_pixels, sizeof(uint16_t));
    uint16_t *parent = detect_malloc(MAX_LABELS * sizeof(uint16_t));
    component_stats_t *stats = detect_calloc(MAX_LABELS, sizeof(component_stats_t));
    if (!labels || !parent || !stats) {
        free(labels);
        free(parent);
        free(stats);
        return;
    }

    uf_init(parent, MAX_LABELS);
    for (uint16_t i = 0; i < MAX_LABELS; i++) {
        stats[i].min_x = img_w;
        stats[i].min_y = img_h;
    }

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

    for (uint16_t y = 0; y < img_h; y++) {
        for (uint16_t x = 0; x < img_w; x++) {
            size_t idx = (size_t)y * img_w + x;
            uint16_t raw_label = labels[idx];
            if (raw_label == 0) {
                continue;
            }

            uint16_t root = uf_find(parent, raw_label);
            component_stats_t *s = &stats[root];
            stats_add_pixel(s, x, y);
        }
    }

    uint32_t min_line_area = total_pixels / 900;
    if (min_line_area < 35) {
        min_line_area = 35;
    }
    component_stats_t *left = NULL;
    component_stats_t *right = NULL;
    for (uint16_t i = 1; i < next_label; i++) {
        if (uf_find(parent, i) != i) {
            continue;
        }

        component_stats_t *s = &stats[i];
        if (s->count < min_line_area ||
            s->min_x > s->max_x ||
            s->min_y > s->max_y) {
            continue;
        }

        uint16_t bbox_w = s->max_x - s->min_x + 1;
        uint16_t bbox_h = s->max_y - s->min_y + 1;
        if (bbox_h < img_h / 5 ||
            bbox_w > img_w / 5 ||
            bbox_h < bbox_w * 2) {
            continue;
        }

        uint16_t center_x = (uint16_t)(s->sum_x / s->count);
        if (!left || center_x < (uint16_t)(left->sum_x / left->count)) {
            left = s;
        }
        if (!right || center_x > (uint16_t)(right->sum_x / right->count)) {
            right = s;
        }
    }

    if (left && right && left != right) {
        uint16_t left_center = (uint16_t)(left->sum_x / left->count);
        uint16_t right_center = (uint16_t)(right->sum_x / right->count);
        if (right_center > left_center &&
            right_center - left_center > img_w / 4) {
            uint16_t min_x = left->min_x < right->min_x ? left->min_x : right->min_x;
            uint16_t min_y = left->min_y < right->min_y ? left->min_y : right->min_y;
            uint16_t max_x = left->max_x > right->max_x ? left->max_x : right->max_x;
            uint16_t max_y = left->max_y > right->max_y ? left->max_y : right->max_y;

            board->found = true;
            board->center_x = (uint16_t)((min_x + max_x) / 2);
            board->center_y = (uint16_t)((min_y + max_y) / 2);
            board->bbox_x = min_x;
            board->bbox_y = min_y;
            board->bbox_w = max_x - min_x + 1;
            board->bbox_h = max_y - min_y + 1;
            board->tl_x = (float)min_x;
            board->tl_y = (float)min_y;
            board->tr_x = (float)max_x;
            board->tr_y = (float)min_y;
            board->br_x = (float)max_x;
            board->br_y = (float)max_y;
            board->bl_x = (float)min_x;
            board->bl_y = (float)max_y;
            board->area_px = left->count + right->count;
            update_board_bbox_from_corners(board, img_w, img_h);
        }
    }

    free(labels);
    free(parent);
    free(stats);
}

static void stats_init(component_stats_t *s, uint16_t img_w, uint16_t img_h)
{
    memset(s, 0, sizeof(*s));
    s->min_x = img_w;
    s->min_y = img_h;
}

static void stats_add_pixel(component_stats_t *s, uint16_t x, uint16_t y)
{
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

static bool append_fruit_result(fruit_detect_result_t *result,
                                uint16_t img_w,
                                uint16_t img_h,
                                component_stats_t *s,
                                uint32_t min_area,
                                bool strict_shape)
{
    if (result->count >= FRUIT_DETECT_MAX_FRUITS ||
        s->count < min_area ||
        s->min_x > s->max_x ||
        s->min_y > s->max_y) {
        return false;
    }

    uint16_t bbox_w = s->max_x - s->min_x + 1;
    uint16_t bbox_h = s->max_y - s->min_y + 1;
    uint16_t min_side = bbox_w < bbox_h ? bbox_w : bbox_h;
    uint16_t max_side = bbox_w > bbox_h ? bbox_w : bbox_h;
    uint32_t bbox_area = (uint32_t)bbox_w * bbox_h;
    uint32_t min_density = strict_shape ? 22 : 16;
    uint32_t max_aspect_x10 = strict_shape ? 20 : 26;

    if (min_side < 12 ||
        max_side * 10 > min_side * max_aspect_x10 ||
        s->count * 100 < bbox_area * min_density) {
        return false;
    }

    uint16_t area_diameter = isqrt_u32((uint32_t)((s->count * 400UL + 157UL) / 314UL));
    if (area_diameter < MIN_DIAMETER_PX || max_side < MIN_DIAMETER_PX) {
        return false;
    }

    uint16_t diameter = (area_diameter * 108U) / 100U;
    diameter = clamp_u16(diameter, MIN_DIAMETER_PX, max_side);

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
    f->size_grade = (diameter - 1 > LARGE_MIN_X_SPAN_PX) ? FRUIT_GRADE_LARGE : FRUIT_GRADE_SMALL;

    return true;
}

static uint8_t try_axis_split(fruit_detect_result_t *result,
                              uint16_t *labels,
                              component_stats_t *stats,
                              uint16_t root,
                              uint16_t img_w,
                              uint16_t img_h,
                              uint32_t min_area)
{
    component_stats_t *s = &stats[root];
    uint16_t bbox_w = s->max_x - s->min_x + 1;
    uint16_t bbox_h = s->max_y - s->min_y + 1;
    bool split_x;

    if (bbox_w * 10 >= bbox_h * 15) {
        split_x = true;
    } else if (bbox_h * 10 >= bbox_w * 15) {
        split_x = false;
    } else {
        return 0;
    }

    uint16_t axis_len = split_x ? bbox_w : bbox_h;
    uint16_t *projection = detect_calloc(axis_len, sizeof(uint16_t));
    if (!projection) {
        return 0;
    }

    for (uint16_t y = s->min_y; y <= s->max_y; y++) {
        for (uint16_t x = s->min_x; x <= s->max_x; x++) {
            if (labels[(size_t)y * img_w + x] != root) {
                continue;
            }
            uint16_t axis = split_x ? (x - s->min_x) : (y - s->min_y);
            projection[axis]++;
        }
    }

    uint16_t search_start = axis_len / 4;
    uint16_t search_end = axis_len - search_start;
    uint16_t valley = search_start;
    uint16_t valley_count = UINT16_MAX;

    for (uint16_t i = search_start; i < search_end; i++) {
        if (projection[i] < valley_count) {
            valley_count = projection[i];
            valley = i;
        }
    }

    uint16_t left_max = 0;
    uint16_t right_max = 0;
    for (uint16_t i = 0; i < valley; i++) {
        if (projection[i] > left_max) {
            left_max = projection[i];
        }
    }
    for (uint16_t i = valley + 1; i < axis_len; i++) {
        if (projection[i] > right_max) {
            right_max = projection[i];
        }
    }

    free(projection);

    uint16_t side_max = left_max < right_max ? left_max : right_max;
    if (side_max == 0 || valley_count * 100 > side_max * 58) {
        return 0;
    }

    component_stats_t a;
    component_stats_t b;
    stats_init(&a, img_w, img_h);
    stats_init(&b, img_w, img_h);

    uint16_t split_coord = split_x ? (s->min_x + valley) : (s->min_y + valley);

    for (uint16_t y = s->min_y; y <= s->max_y; y++) {
        for (uint16_t x = s->min_x; x <= s->max_x; x++) {
            if (labels[(size_t)y * img_w + x] != root) {
                continue;
            }
            if ((split_x && x <= split_coord) || (!split_x && y <= split_coord)) {
                stats_add_pixel(&a, x, y);
            } else {
                stats_add_pixel(&b, x, y);
            }
        }
    }

    uint8_t before = result->count;
    uint32_t split_min_area = min_area / 2;
    if (split_min_area < 260) {
        split_min_area = 260;
    }

    bool ok_a = append_fruit_result(result, img_w, img_h, &a, split_min_area, false);
    bool ok_b = append_fruit_result(result, img_w, img_h, &b, split_min_area, false);
    if (ok_a && ok_b) {
        return 2;
    }

    result->count = before;
    return 0;
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
    uint8_t *board_mask = detect_malloc(total_pixels);
    uint8_t *gray = detect_malloc(total_pixels);
    if (!mask || !board_mask || !gray) {
        ESP_LOGE(TAG, "No memory for mask, need %u bytes", (unsigned int)total_pixels);
        free(rgb);
        free(mask);
        free(board_mask);
        free(gray);
        return ESP_ERR_NO_MEM;
    }

    for (size_t i = 0; i < total_pixels; i++) {
        uint8_t r = rgb[i * 3];
        uint8_t g = rgb[i * 3 + 1];
        uint8_t b = rgb[i * 3 + 2];
        bool citrus = is_citrus_pixel(r, g, b);
        mask[i] = citrus ? 1 : 0;
        gray[i] = rgb_luma(r, g, b);
        board_mask[i] = (!citrus && is_black_reference_pixel(r, g, b)) ? 1 : 0;
    }

    for (uint16_t y = 4; y + 4 < img_h; y++) {
        for (uint16_t x = 4; x + 4 < img_w; x++) {
            size_t idx = (size_t)y * img_w + x;
            uint8_t center = gray[idx];
            uint8_t left = gray[(size_t)y * img_w + x - 4];
            uint8_t right = gray[(size_t)y * img_w + x + 4];
            uint8_t up = gray[(size_t)(y - 4) * img_w + x];
            uint8_t down = gray[(size_t)(y + 4) * img_w + x];
            uint8_t horizontal_bg = (uint8_t)(((int)left + (int)right) / 2);
            uint8_t vertical_bg = (uint8_t)(((int)up + (int)down) / 2);
            bool vertical_line = horizontal_bg > center + 7;
            bool horizontal_line = vertical_bg > center + 7;
            if ((vertical_line || horizontal_line) && center < 190 && !mask[idx]) {
                board_mask[idx] = 1;
            }
        }
    }

    free(rgb);
    free(gray);

    uint8_t *scratch = detect_malloc(total_pixels);
    if (!scratch) {
        ESP_LOGE(TAG, "No memory for mask scratch, need %u bytes", (unsigned int)total_pixels);
        free(mask);
        free(board_mask);
        return ESP_ERR_NO_MEM;
    }
    detect_board_from_mask(board_mask, img_w, img_h, &result->board);
    free(board_mask);

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
        uint32_t dynamic_min_area = total_pixels / 500;
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

        if (try_axis_split(result, labels, stats, i, img_w, img_h, dynamic_min_area) == 2) {
            continue;
        }

        if (min_side < 12 ||
            max_side > min_side * 2 ||
            s->count * 100 < bbox_area * 22 ||
            circularity100 < 32) {
            continue;
        }

        append_fruit_result(result, img_w, img_h, s, dynamic_min_area, true);
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
