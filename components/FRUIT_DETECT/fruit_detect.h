#ifndef __FRUIT_DETECT_H_
#define __FRUIT_DETECT_H_

#include <stdint.h>

#include "esp_camera.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FRUIT_DETECT_MAX_FRUITS 10

typedef enum {
    FRUIT_GRADE_SMALL = 0,
    FRUIT_GRADE_MEDIUM,
    FRUIT_GRADE_LARGE,
} fruit_grade_t;

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
    uint16_t image_width;
    uint16_t image_height;
    uint8_t count;
    fruit_info_t fruits[FRUIT_DETECT_MAX_FRUITS];
} fruit_detect_result_t;

esp_err_t fruit_detect_init(void);
esp_err_t fruit_detect_process(camera_fb_t *fb, fruit_detect_result_t *result);
const char *fruit_grade_label(fruit_grade_t grade);

#ifdef __cplusplus
}
#endif

#endif
