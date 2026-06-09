#ifndef __M0_UART_H_
#define __M0_UART_H_

#include "esp_err.h"
#include "fruit_detect.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Default wiring:
 *   ESP32-S3 GPIO41 (UART1 TX) -> M0 UART RX
 *   GND shared between boards
 *
 * Output frame, 4 binary bytes per detection:
 *   [0] 0xAA       frame header
 *   [1] has_fruit  0=no fruit, 1=fruit found
 *   [2] x          first fruit center_x, clamped to 0..255
 *   [3] y          first fruit center_y, clamped to 0..255
 *
 * If has_fruit is 0, x and y are both 0.
 */
#define M0_UART_BAUD_RATE 115200

esp_err_t m0_uart_init(void);
esp_err_t m0_uart_send_result(const fruit_detect_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
