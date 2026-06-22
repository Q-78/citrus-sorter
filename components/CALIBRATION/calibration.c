#include "calibration.h"

#include <math.h>

/*
 * Pixel -> world homography fitted from 6 calibration points:
 *   world (0,0)   -> pixel (87,39)
 *   world (25,0)  -> pixel (211,54)
 *   world (5,20)  -> pixel (102,144)
 *   world (15,20) -> pixel (157,149)
 *   world (0,30)  -> pixel (71,202)
 *   world (25,30) -> pixel (199,205)
 */
#define H00 0.1763891577f
#define H01 0.0190288364f
#define H02 (-16.1725048390f)
#define H10 (-0.0234265476f)
#define H11 0.1823029910f
#define H12 (-4.8911018994f)
#define H20 (-0.0006311496f)
#define H21 0.0002339553f

bool calibration_pixel_to_world(uint16_t pixel_x, uint16_t pixel_y, world_coord_t *world)
{
    if (!world) {
        return false;
    }

    float u = (float)pixel_x;
    float v = (float)pixel_y;
    float denom = H20 * u + H21 * v + 1.0f;
    if (fabsf(denom) < 0.000001f) {
        return false;
    }

    world->x = (H00 * u + H01 * v + H02) / denom;
    world->y = (H10 * u + H11 * v + H12) / denom;
    return true;
}

uint8_t calibration_world_to_u8(float value)
{
    if (value <= 0.0f) {
        return 0;
    }
    if (value >= 255.0f) {
        return 255;
    }
    return (uint8_t)(value + 0.5f);
}
