#include "camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_log.h"

static const char *TAG = "camera";

// ================= OV5640 / OV2640 Camera Pin Map =================
// Based on Freenove ESP32-S3 WROOM Pinout camera labels

#define CAM_PIN_PWDN    GPIO_NUM_NC
#define CAM_PIN_RESET   GPIO_NUM_NC

#define CAM_PIN_XCLK    GPIO_NUM_15
#define CAM_PIN_SIOD    GPIO_NUM_4
#define CAM_PIN_SIOC    GPIO_NUM_5

#define CAM_PIN_D7      GPIO_NUM_16    // CAM_Y9
#define CAM_PIN_D6      GPIO_NUM_17    // CAM_Y8
#define CAM_PIN_D5      GPIO_NUM_18    // CAM_Y7
#define CAM_PIN_D4      GPIO_NUM_12    // CAM_Y6
#define CAM_PIN_D3      GPIO_NUM_10    // CAM_Y5
#define CAM_PIN_D2      GPIO_NUM_8     // CAM_Y4
#define CAM_PIN_D1      GPIO_NUM_9     // CAM_Y3
#define CAM_PIN_D0      GPIO_NUM_11    // CAM_Y2

#define CAM_PIN_VSYNC   GPIO_NUM_6
#define CAM_PIN_HREF    GPIO_NUM_7
#define CAM_PIN_PCLK    GPIO_NUM_13

static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    // 【修改】调试阶段建议先降到 20MHz，更稳定；
    // 如果你的 OV5640 模块自带外部时钟，CAM_PIN_XCLK 才可以为 GPIO_NUM_NC。
    .xclk_freq_hz = 10000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    // 【修改】网页直接显示 JPEG，所以采集格式用 JPEG。
    .pixel_format = PIXFORMAT_JPEG,

    // 【修改】调试阶段先用 VGA/QVGA，确认采集成功后再提高到 UXGA。
    .frame_size = FRAMESIZE_QVGA,

    .jpeg_quality = 12,
    .fb_count = 1,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .fb_location = CAMERA_FB_IN_PSRAM,
};

static void camera_gpio_output_if_valid(gpio_num_t pin)
{
    if (pin == GPIO_NUM_NC) {
        return;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

esp_err_t camera_init(void)
{
    if (CAM_PIN_XCLK == GPIO_NUM_NC) {
        ESP_LOGW(TAG, "CAM_PIN_XCLK is GPIO_NUM_NC. Make sure the OV5640 module has an external XCLK/clock source, otherwise capture will fail.");
    }

    // 【修改】只有 PWDN/RESET 引脚有效时才手动操作，避免 GPIO_NUM_NC 导致错误。
    camera_gpio_output_if_valid(CAM_PIN_PWDN);
    camera_gpio_output_if_valid(CAM_PIN_RESET);

    /*
    if (CAM_PIN_PWDN != GPIO_NUM_NC) {
        gpio_set_level(CAM_PIN_PWDN, 0);  // power up
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (CAM_PIN_RESET != GPIO_NUM_NC) {
        gpio_set_level(CAM_PIN_RESET, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(CAM_PIN_RESET, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    */
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, FRAMESIZE_QVGA);
        s->set_quality(s, 12);
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
    }

    ESP_LOGI(TAG, "Camera init OK");
    return ESP_OK;
}

camera_fb_t *camera_capture(void)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return NULL;
    }

    ESP_LOGI(TAG, "Captured frame: %ux%u, len=%u, format=%d",
             fb->width, fb->height, fb->len, fb->format);

    return fb;
}

void camera_return(camera_fb_t *fb)
{
    if (fb) {
        esp_camera_fb_return(fb);
    }
}