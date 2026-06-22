#ifndef __CAMERA_WEB_SERVER_H_
#define __CAMERA_WEB_SERVER_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void start_camera_web_server(void);
esp_err_t start_camera_detection_task(void);

#ifdef __cplusplus
}
#endif

#endif
