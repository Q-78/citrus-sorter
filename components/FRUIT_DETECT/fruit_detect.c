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
#define LARGE_MIN_REFERENCE_RATIO_X1000 407U
#define BOARD_MIN_CORNER_ANGLE_DEG 65.0f
#define BOARD_MAX_CORNER_ANGLE_DEG 115.0f

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
static float distance_f(float ax, float ay, float bx, float by);

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

    if (maxc < 65 || delta < 28) {
        return false;
    }

    int saturation = delta * 255 / maxc;
    int hue = rgb_hue_deg(r, g, b);

    bool saturated_or_highlight =
        saturation >= 140 ||
        (maxc >= 225 && saturation >= 95 && r >= g + 45 && r >= b + 80);

    return saturated_or_highlight &&
           hue >= 10 && hue <= 82 &&
           r >= 75 &&
           g >= 30 &&
           r >= b + 38 &&
           g >= b + 4 &&
           b * 100 <= maxc * 72 &&
           g * 100 >= r * 22 &&
           g * 100 <= r * 125;
}

static bool is_blue_reference_pixel(uint8_t r, uint8_t g, uint8_t b)
{
    int maxc = max3(r, g, b);
    int minc = min3(r, g, b);
    int delta = maxc - minc;

    if (maxc < 55 || delta < 18 || b < 65) {
        return false;
    }

    int saturation = delta * 255 / maxc;
    int hue = rgb_hue_deg(r, g, b);

    if (b < r + 10 || b + 12 < g) {
        return false;
    }

    if (saturation >= 40 && hue >= 175 && hue <= 255) {
        return true;
    }

    return maxc >= 165 && b >= r + 18 && b + 8 >= g;
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

static void close_mask_3x3(uint8_t *mask, uint8_t *scratch, uint16_t width, uint16_t height)
{
    memset(scratch, 0, (size_t)width * height);

    for (uint16_t y = 1; y + 1 < height; y++) {
        for (uint16_t x = 1; x + 1 < width; x++) {
            bool hit = false;
            for (int dy = -1; dy <= 1 && !hit; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (mask[(size_t)(y + dy) * width + (x + dx)]) {
                        hit = true;
                        break;
                    }
                }
            }
            scratch[(size_t)y * width + x] = hit ? 1 : 0;
        }
    }

    memset(mask, 0, (size_t)width * height);

    for (uint16_t y = 1; y + 1 < height; y++) {
        for (uint16_t x = 1; x + 1 < width; x++) {
            uint8_t count = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    count += scratch[(size_t)(y + dy) * width + (x + dx)] ? 1 : 0;
                }
            }
            mask[(size_t)y * width + x] = count >= 5 ? 1 : 0;
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

typedef struct {
    bool found;
    float x;
    float y;
    uint16_t min_x;
    uint16_t max_x;
    uint16_t min_y;
    uint16_t max_y;
    uint32_t area;
    uint32_t score;
} blue_marker_t;

static float cross_points(float ax, float ay, float bx, float by, float cx, float cy)
{
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

static float quad_area2(const board_info_t *board)
{
    float area = 0.0f;
    area += board->tl_x * board->tr_y - board->tl_y * board->tr_x;
    area += board->tr_x * board->br_y - board->tr_y * board->br_x;
    area += board->br_x * board->bl_y - board->br_y * board->bl_x;
    area += board->bl_x * board->tl_y - board->bl_y * board->tl_x;
    return fabsf(area);
}

static bool board_corner_angle_valid(float prev_x,
                                     float prev_y,
                                     float corner_x,
                                     float corner_y,
                                     float next_x,
                                     float next_y)
{
    float ax = prev_x - corner_x;
    float ay = prev_y - corner_y;
    float bx = next_x - corner_x;
    float by = next_y - corner_y;
    float len_a = sqrtf(ax * ax + ay * ay);
    float len_b = sqrtf(bx * bx + by * by);
    if (len_a < 1.0f || len_b < 1.0f) {
        return false;
    }

    float cos_angle = (ax * bx + ay * by) / (len_a * len_b);
    float min_cos = cosf(BOARD_MAX_CORNER_ANGLE_DEG * (float)M_PI / 180.0f);
    float max_cos = cosf(BOARD_MIN_CORNER_ANGLE_DEG * (float)M_PI / 180.0f);
    return cos_angle >= min_cos && cos_angle <= max_cos;
}

bool fruit_detect_board_geometry_valid(const board_info_t *board)
{
    if (!board || !board->found) {
        return false;
    }

    return board_corner_angle_valid(board->bl_x, board->bl_y,
                                    board->tl_x, board->tl_y,
                                    board->tr_x, board->tr_y) &&
           board_corner_angle_valid(board->tl_x, board->tl_y,
                                    board->tr_x, board->tr_y,
                                    board->br_x, board->br_y) &&
           board_corner_angle_valid(board->tr_x, board->tr_y,
                                    board->br_x, board->br_y,
                                    board->bl_x, board->bl_y) &&
           board_corner_angle_valid(board->br_x, board->br_y,
                                    board->bl_x, board->bl_y,
                                    board->tl_x, board->tl_y);
}

static bool blue_marker_board_geometry_ok(const board_info_t *board,
                                          uint16_t img_w,
                                          uint16_t img_h)
{
    float c1 = cross_points(board->tl_x, board->tl_y, board->tr_x, board->tr_y,
                            board->br_x, board->br_y);
    float c2 = cross_points(board->tr_x, board->tr_y, board->br_x, board->br_y,
                            board->bl_x, board->bl_y);
    float c3 = cross_points(board->br_x, board->br_y, board->bl_x, board->bl_y,
                            board->tl_x, board->tl_y);
    float c4 = cross_points(board->bl_x, board->bl_y, board->tl_x, board->tl_y,
                            board->tr_x, board->tr_y);
    bool all_pos = c1 > 0.0f && c2 > 0.0f && c3 > 0.0f && c4 > 0.0f;
    bool all_neg = c1 < 0.0f && c2 < 0.0f && c3 < 0.0f && c4 < 0.0f;
    if (!all_pos && !all_neg) {
        return false;
    }

    float top_w = distance_f(board->tl_x, board->tl_y, board->tr_x, board->tr_y);
    float bottom_w = distance_f(board->bl_x, board->bl_y, board->br_x, board->br_y);
    float left_h = distance_f(board->tl_x, board->tl_y, board->bl_x, board->bl_y);
    float right_h = distance_f(board->tr_x, board->tr_y, board->br_x, board->br_y);
    float min_side = fminf(fminf(top_w, bottom_w), fminf(left_h, right_h));
    if (min_side < 35.0f) {
        return false;
    }

    float width_ratio = fmaxf(top_w, bottom_w) / fmaxf(1.0f, fminf(top_w, bottom_w));
    float height_ratio = fmaxf(left_h, right_h) / fmaxf(1.0f, fminf(left_h, right_h));
    if (width_ratio > 1.55f || height_ratio > 1.45f) {
        return false;
    }

    float avg_width = (top_w + bottom_w) * 0.5f;
    float avg_height = (left_h + right_h) * 0.5f;
    float aspect = avg_width / fmaxf(1.0f, avg_height);
    if (aspect < 1.45f || aspect > 4.2f) {
        return false;
    }

    if (!fruit_detect_board_geometry_valid(board)) {
        return false;
    }

    float area2 = quad_area2(board);
    float min_area2 = (float)((uint32_t)img_w * img_h) * 0.06f;
    return area2 >= min_area2;
}

static bool blue_marker_candidate_in_safe_area(const component_stats_t *s,
                                               float cx,
                                               float cy,
                                               uint16_t img_w,
                                               uint16_t img_h)
{
    if (!s) {
        return false;
    }

    uint16_t margin_x = img_w / 45;
    uint16_t margin_y = img_h / 45;
    if (margin_x < 18) {
        margin_x = 18;
    }
    if (margin_y < 14) {
        margin_y = 14;
    }

    if (s->min_x <= 1 || s->min_y <= 1 ||
        s->max_x + 2 >= img_w || s->max_y + 2 >= img_h) {
        return false;
    }

    return cx >= (float)margin_x &&
           cy >= (float)margin_y &&
           cx <= (float)(img_w - 1 - margin_x) &&
           cy <= (float)(img_h - 1 - margin_y);
}

static float blue_marker_board_score(const board_info_t *board,
                                     const blue_marker_t markers[4])
{
    float top_w = distance_f(board->tl_x, board->tl_y, board->tr_x, board->tr_y);
    float bottom_w = distance_f(board->bl_x, board->bl_y, board->br_x, board->br_y);
    float left_h = distance_f(board->tl_x, board->tl_y, board->bl_x, board->bl_y);
    float right_h = distance_f(board->tr_x, board->tr_y, board->br_x, board->br_y);
    float width_balance = fabsf(top_w - bottom_w) / fmaxf(1.0f, fmaxf(top_w, bottom_w));
    float height_balance = fabsf(left_h - right_h) / fmaxf(1.0f, fmaxf(left_h, right_h));
    uint32_t marker_score = markers[0].score + markers[1].score +
                            markers[2].score + markers[3].score;
    return quad_area2(board) + (float)marker_score * 12.0f -
           (width_balance + height_balance) * 9000.0f;
}

static bool assign_blue_marker_corners(const blue_marker_t markers[4],
                                       board_info_t *board)
{
    if (!markers || !board) {
        return false;
    }

    uint8_t tl = 0;
    uint8_t br = 0;
    uint8_t tr = 0;
    uint8_t bl = 0;
    float min_sum = markers[0].x + markers[0].y;
    float max_sum = min_sum;
    float max_diff = markers[0].x - markers[0].y;
    float min_diff = max_diff;

    for (uint8_t i = 1; i < 4; i++) {
        float sum = markers[i].x + markers[i].y;
        float diff = markers[i].x - markers[i].y;
        if (sum < min_sum) {
            min_sum = sum;
            tl = i;
        }
        if (sum > max_sum) {
            max_sum = sum;
            br = i;
        }
        if (diff > max_diff) {
            max_diff = diff;
            tr = i;
        }
        if (diff < min_diff) {
            min_diff = diff;
            bl = i;
        }
    }

    if (tl == tr || tl == br || tl == bl || tr == br || tr == bl || br == bl) {
        return false;
    }

    float top_w = distance_f(markers[tl].x, markers[tl].y, markers[tr].x, markers[tr].y);
    float bottom_w = distance_f(markers[bl].x, markers[bl].y, markers[br].x, markers[br].y);
    float left_h = distance_f(markers[tl].x, markers[tl].y, markers[bl].x, markers[bl].y);
    float right_h = distance_f(markers[tr].x, markers[tr].y, markers[br].x, markers[br].y);
    if (top_w < 30.0f || bottom_w < 30.0f || left_h < 30.0f || right_h < 30.0f) {
        return false;
    }

    board->found = true;
    board->reference_mode = BOARD_REFERENCE_BLUE_DOTS;
    board->tl_x = markers[tl].x;
    board->tl_y = markers[tl].y;
    board->tr_x = markers[tr].x;
    board->tr_y = markers[tr].y;
    board->br_x = markers[br].x;
    board->br_y = markers[br].y;
    board->bl_x = markers[bl].x;
    board->bl_y = markers[bl].y;
    board->area_px = markers[0].area + markers[1].area + markers[2].area + markers[3].area;
    return true;
}

static bool select_blue_marker_board(const blue_marker_t *candidates,
                                     uint8_t candidate_count,
                                     uint16_t img_w,
                                     uint16_t img_h,
                                     board_info_t *board)
{
    if (!candidates || candidate_count < 4 || !board) {
        return false;
    }

    bool found = false;
    float best_score = -1000000000.0f;
    board_info_t best_board = {0};

    for (uint8_t a = 0; a + 3 < candidate_count; a++) {
        for (uint8_t b = a + 1; b + 2 < candidate_count; b++) {
            for (uint8_t c = b + 1; c + 1 < candidate_count; c++) {
                for (uint8_t d = c + 1; d < candidate_count; d++) {
                    blue_marker_t quad[4] = {
                        candidates[a], candidates[b], candidates[c], candidates[d],
                    };
                    board_info_t candidate_board = {0};
                    if (!assign_blue_marker_corners(quad, &candidate_board) ||
                        !blue_marker_board_geometry_ok(&candidate_board, img_w, img_h)) {
                        continue;
                    }

                    float score = blue_marker_board_score(&candidate_board, quad);
                    if (!found || score > best_score) {
                        found = true;
                        best_score = score;
                        best_board = candidate_board;
                    }
                }
            }
        }
    }

    if (!found) {
        return false;
    }

    *board = best_board;
    return true;
}

static bool detect_board_from_blue_markers(uint8_t *mask,
                                           uint16_t img_w,
                                           uint16_t img_h,
                                           board_info_t *board)
{
    if (!mask || !board) {
        return false;
    }

    uint16_t *labels = detect_calloc((size_t)img_w * img_h, sizeof(uint16_t));
    uint16_t *parent = detect_malloc(MAX_LABELS * sizeof(uint16_t));
    component_stats_t *stats = detect_calloc(MAX_LABELS, sizeof(component_stats_t));
    if (!labels || !parent || !stats) {
        free(labels);
        free(parent);
        free(stats);
        return false;
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
            labels[idx] = root;
            stats_add_pixel(&stats[root], x, y);
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

    blue_marker_t candidates[12] = {0};
    uint8_t candidate_count = 0;
    uint32_t min_area = ((uint32_t)img_w * img_h) / 12000U;
    if (min_area < 10) {
        min_area = 10;
    }
    uint32_t max_area = ((uint32_t)img_w * img_h) / 550U;
    if (max_area < 80) {
        max_area = 80;
    }

    for (uint16_t i = 1; i < next_label; i++) {
        if (uf_find(parent, i) != i) {
            continue;
        }

        component_stats_t *s = &stats[i];
        if (s->count < min_area || s->count > max_area ||
            s->perimeter == 0 ||
            s->min_x > s->max_x || s->min_y > s->max_y) {
            continue;
        }

        uint16_t bbox_w = s->max_x - s->min_x + 1;
        uint16_t bbox_h = s->max_y - s->min_y + 1;
        uint16_t min_side = bbox_w < bbox_h ? bbox_w : bbox_h;
        uint16_t max_side = bbox_w > bbox_h ? bbox_w : bbox_h;
        if (min_side < 4 || max_side * 10 > min_side * 28) {
            continue;
        }

        uint32_t circularity100 =
            (uint32_t)((1256ULL * s->count) / ((uint64_t)s->perimeter * s->perimeter));
        if (circularity100 < 24) {
            continue;
        }

        uint32_t bbox_area = (uint32_t)bbox_w * bbox_h;
        uint32_t fill100 = (s->count * 100U) / bbox_area;
        uint32_t score = s->count * 3U + circularity100 * 3U + fill100;

        float cx = (float)s->sum_x / (float)s->count;
        float cy = (float)s->sum_y / (float)s->count;
        if (!blue_marker_candidate_in_safe_area(s, cx, cy, img_w, img_h)) {
            continue;
        }

        blue_marker_t candidate = {
            .found = true,
            .x = cx,
            .y = cy,
            .min_x = s->min_x,
            .max_x = s->max_x,
            .min_y = s->min_y,
            .max_y = s->max_y,
            .area = s->count,
            .score = score,
        };

        for (uint8_t slot = 0; slot < 12; slot++) {
            if (!candidates[slot].found || candidate.score > candidates[slot].score) {
                for (int8_t j = 11; j > (int8_t)slot; j--) {
                    candidates[j] = candidates[j - 1];
                }
                candidates[slot] = candidate;
                if (candidate_count < 12) {
                    candidate_count++;
                }
                break;
            }
        }
    }

    bool ok = select_blue_marker_board(candidates, candidate_count, img_w, img_h, board);
    if (ok) {
        update_board_bbox_from_corners(board, img_w, img_h);
    }

    free(labels);
    free(parent);
    free(stats);
    return ok;
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
    *relative_x = u * 100.0f;
    *relative_y = v * 100.0f;
    return true;
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

    uint16_t diameter = clamp_u16(area_diameter, MIN_DIAMETER_PX, max_side);

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
    f->size_grade = FRUIT_GRADE_SMALL;

    return true;
}

static float board_reference_short_side_px(const board_info_t *board)
{
    if (!board || !board->found) {
        return 0.0f;
    }

    float top_w = distance_f(board->tl_x, board->tl_y, board->tr_x, board->tr_y);
    float bottom_w = distance_f(board->bl_x, board->bl_y, board->br_x, board->br_y);
    float left_h = distance_f(board->tl_x, board->tl_y, board->bl_x, board->bl_y);
    float right_h = distance_f(board->tr_x, board->tr_y, board->br_x, board->br_y);
    float avg_width = (top_w + bottom_w) * 0.5f;
    float avg_height = (left_h + right_h) * 0.5f;

    return avg_width < avg_height ? avg_width : avg_height;
}

static void update_fruit_size_grades(fruit_detect_result_t *result)
{
    if (!result) {
        return;
    }

    float reference_short_side = board_reference_short_side_px(&result->board);
    for (uint8_t i = 0; i < result->count; i++) {
        fruit_info_t *fruit = &result->fruits[i];
        if (reference_short_side > 1.0f) {
            uint32_t ratio_x1000 =
                (uint32_t)(((float)fruit->diameter_px * 1000.0f) / reference_short_side);
            fruit->size_grade = ratio_x1000 >= LARGE_MIN_REFERENCE_RATIO_X1000
                                    ? FRUIT_GRADE_LARGE
                                    : FRUIT_GRADE_SMALL;
        } else {
            fruit->size_grade = FRUIT_GRADE_SMALL;
        }
    }
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
    uint8_t *blue_mask = detect_malloc(total_pixels);
    if (!mask || !blue_mask) {
        ESP_LOGE(TAG, "No memory for mask, need %u bytes", (unsigned int)total_pixels);
        free(rgb);
        free(mask);
        free(blue_mask);
        return ESP_ERR_NO_MEM;
    }

    for (size_t i = 0; i < total_pixels; i++) {
        uint8_t r = rgb[i * 3];
        uint8_t g = rgb[i * 3 + 1];
        uint8_t b = rgb[i * 3 + 2];
        bool citrus = is_citrus_pixel(r, g, b);
        bool blue_reference = is_blue_reference_pixel(r, g, b);
        mask[i] = citrus ? 1 : 0;
        blue_mask[i] = blue_reference ? 1 : 0;
    }

    free(rgb);

    uint8_t *scratch = detect_malloc(total_pixels);
    if (!scratch) {
        ESP_LOGE(TAG, "No memory for mask scratch, need %u bytes", (unsigned int)total_pixels);
        free(mask);
        free(blue_mask);
        return ESP_ERR_NO_MEM;
    }

    close_mask_3x3(blue_mask, scratch, img_w, img_h);
    detect_board_from_blue_markers(blue_mask, img_w, img_h, &result->board);
    free(blue_mask);

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

    update_fruit_size_grades(result);

    ESP_LOGI(TAG, "Detected %u fruit(s) in %ux%u frame",
             result->count, img_w, img_h);
    return ESP_OK;
}
