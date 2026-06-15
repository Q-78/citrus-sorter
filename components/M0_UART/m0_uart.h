#ifndef __M0_UART_H_
#define __M0_UART_H_

#include <stdint.h>

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
 * Output frame, 5 binary bytes per detection:
 *   [0] 0xAA       frame header
 *   [1] has_fruit  0=no fruit, 1=fruit found
 *   [2] grade      0=small, 1=large
 *   [3] x          first fruit center_x, clamped to 0..255
 *   [4] y          first fruit center_y, clamped to 0..255
 *
 * If has_fruit is 0, grade, x, and y are 0.
 */
#define M0_UART_BAUD_RATE 115200

typedef struct {
    uint8_t has_fruit;
    uint8_t grade;
    uint8_t x;
    uint8_t y;
} m0_uart_payload_t;

esp_err_t m0_uart_init(void);
esp_err_t m0_uart_build_payload(const fruit_detect_result_t *result, m0_uart_payload_t *payload);
esp_err_t m0_uart_send_result(const fruit_detect_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
