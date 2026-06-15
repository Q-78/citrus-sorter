#include "m0_uart.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "m0_uart";

#define M0_UART_PORT UART_NUM_1
#define M0_UART_TX_PIN GPIO_NUM_41
#define M0_UART_BUF_SIZE 512
#define M0_UART_FRAME_HEADER 0xAA
#define M0_UART_FRAME_SIZE 5
/* Loopback debug, disabled for normal ESP32 -> M0 communication. */
// #define M0_UART_LOOPBACK_DEBUG 1
// #define M0_UART_RX_PIN GPIO_NUM_42

static bool s_m0_uart_ready;

esp_err_t m0_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = M0_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(M0_UART_PORT, M0_UART_BUF_SIZE, 0, 0, NULL, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uart_driver_install failed: 0x%x", ret);
        return ret;
    }

    ret = uart_param_config(M0_UART_PORT, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: 0x%x", ret);
        return ret;
    }

    ret = uart_set_pin(M0_UART_PORT, M0_UART_TX_PIN, UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: 0x%x", ret);
        return ret;
    }

    s_m0_uart_ready = true;
    ESP_LOGI(TAG, "M0 UART ready: UART%d TX=%d baud=%d",
             M0_UART_PORT, M0_UART_TX_PIN, M0_UART_BAUD_RATE);
    return ESP_OK;
}

static esp_err_t m0_uart_write_frame(const uint8_t *frame, size_t len)
{
    if (!s_m0_uart_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    int written = uart_write_bytes(M0_UART_PORT, frame, len);
    if (written != (int)len) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

/*
 * Loopback debug helper. To use it again:
 *   1. Uncomment M0_UART_LOOPBACK_DEBUG and M0_UART_RX_PIN above.
 *   2. Change uart_driver_install RX buffer size from 0 to M0_UART_BUF_SIZE.
 *   3. Change uart_set_pin RX from UART_PIN_NO_CHANGE to M0_UART_RX_PIN.
 *   4. Short GPIO41 to GPIO42.
 */
#ifdef M0_UART_LOOPBACK_DEBUG
static void m0_uart_try_read_loopback(void)
{
    uint8_t rx[M0_UART_FRAME_SIZE] = {0};
    int len = uart_read_bytes(M0_UART_PORT, rx, sizeof(rx), pdMS_TO_TICKS(20));

    if (len > 0) {
        ESP_LOG_BUFFER_HEX(TAG, rx, len);
    }
}
#endif

static uint8_t clamp_coord_to_u8(uint16_t value)
{
    return value > 255 ? 255 : (uint8_t)value;
}

esp_err_t m0_uart_build_payload(const fruit_detect_result_t *result, m0_uart_payload_t *payload)
{
    if (!result || !payload) {
        return ESP_ERR_INVALID_ARG;
    }

    payload->has_fruit = result->count > 0 ? 1 : 0;
    payload->grade = 0;
    payload->x = 0;
    payload->y = 0;

    if (payload->has_fruit) {
        const fruit_info_t *fruit = &result->fruits[0];
        uint16_t x_min = fruit->bbox_x;
        uint16_t x_max = fruit->bbox_x + fruit->bbox_w - 1;
        uint16_t y_min = fruit->bbox_y;
        uint16_t y_max = fruit->bbox_y + fruit->bbox_h - 1;

        payload->grade = (x_max - x_min > 40 || y_max - y_min > 40) ? 1 : 0;
        payload->x = clamp_coord_to_u8((uint16_t)((x_min + x_max) / 2));
        payload->y = clamp_coord_to_u8((uint16_t)((y_min + y_max) / 2));
    }

    return ESP_OK;
}

esp_err_t m0_uart_send_result(const fruit_detect_result_t *result)
{
    m0_uart_payload_t payload;
    esp_err_t ret = m0_uart_build_payload(result, &payload);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t frame[M0_UART_FRAME_SIZE] = {
        M0_UART_FRAME_HEADER,
        payload.has_fruit,
        payload.grade,
        payload.x,
        payload.y,
    };

    ret = m0_uart_write_frame(frame, sizeof(frame));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sent to M0: AA has=%u grade=%u x=%u y=%u",
                 payload.has_fruit, payload.grade, payload.x, payload.y);
#ifdef M0_UART_LOOPBACK_DEBUG
        uart_wait_tx_done(M0_UART_PORT, pdMS_TO_TICKS(20));
        m0_uart_try_read_loopback();
#endif
    }
    return ret;
}
