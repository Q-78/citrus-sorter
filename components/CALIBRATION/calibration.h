#ifndef __CALIBRATION_H_
#define __CALIBRATION_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
} world_coord_t;

bool calibration_pixel_to_world(uint16_t pixel_x, uint16_t pixel_y, world_coord_t *world);
uint8_t calibration_world_to_u8(float value);

#ifdef __cplusplus
}
#endif

#endif
