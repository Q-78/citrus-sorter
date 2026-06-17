#ifndef __FRUIT_DETECT_H_
#define __FRUIT_DETECT_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_camera.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FRUIT_DETECT_MAX_FRUITS 10

typedef enum {
    FRUIT_GRADE_SMALL = 0,
    FRUIT_GRADE_LARGE = 1,
} fruit_grade_t;

typedef enum {
    BOARD_REFERENCE_NONE = 0,
    BOARD_REFERENCE_BLUE_DOTS = 1,
} board_reference_mode_t;

typedef struct {
    uint16_t center_x;
    uint16_t center_y;
    uint16_t diameter_px;
    uint16_t bbox_x;
    uint16_t bbox_y;
    uint16_t bbox_w;
    uint16_t bbox_h;
    uint32_t area_px;
    fruit_grade_t size_grade;
} fruit_info_t;

typedef struct {
    bool found;
    board_reference_mode_t reference_mode;
    uint16_t center_x;
    uint16_t center_y;
    uint16_t bbox_x;
    uint16_t bbox_y;
    uint16_t bbox_w;
    uint16_t bbox_h;
    float tl_x;
    float tl_y;
    float tr_x;
    float tr_y;
    float br_x;
    float br_y;
    float bl_x;
    float bl_y;
    uint32_t area_px;
} board_info_t;

typedef struct {
    uint16_t image_width;
    uint16_t image_height;
    uint8_t count;
    board_info_t board;
    fruit_info_t fruits[FRUIT_DETECT_MAX_FRUITS];
} fruit_detect_result_t;

esp_err_t fruit_detect_init(void);
esp_err_t fruit_detect_process(camera_fb_t *fb, fruit_detect_result_t *result);
bool fruit_detect_board_relative_coord(const board_info_t *board,
                                       uint16_t pixel_x,
                                       uint16_t pixel_y,
                                       float *relative_x,
                                       float *relative_y);
const char *fruit_grade_label(fruit_grade_t grade);

#ifdef __cplusplus
}
#endif

#endif
