#ifndef __MY_CAMERA_H_
#define __MY_CAMERA_H_

#include "esp_err.h"
#include "esp_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t camera_init(void);
camera_fb_t *camera_capture(void);
void camera_return(camera_fb_t *fb);

#ifdef __cplusplus
}
#endif

#endif