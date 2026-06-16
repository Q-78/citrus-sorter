#ifndef __M0_UART_H_
#define __M0_UART_H_

#include <stdbool.h>
#include <stdint.h>

#include "calibration.h"
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
 * Output frame, 11 binary bytes per detection:
 *   [0]      0xAA       frame header
 *   [1]      has_fruit  0=no fruit, 1=fruit found
 *   [2]      grade      0=small, 1=large
 *   [3..6]   x          first fruit world X, little-endian float32
 *   [7..10]  y          first fruit world Y, little-endian float32
 *
 * If has_fruit is 0, grade, x, and y are 0.
 */
#define M0_UART_BAUD_RATE 115200

typedef struct {
    uint8_t has_fruit;
    uint8_t grade;
    float x;
    float y;
    world_coord_t world;
    bool world_valid;
} m0_uart_payload_t;

esp_err_t m0_uart_init(void);
esp_err_t m0_uart_build_payload(const fruit_detect_result_t *result, m0_uart_payload_t *payload);
esp_err_t m0_uart_send_result(const fruit_detect_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
