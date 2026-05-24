#include <stdio.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "camera.h"
#include "camera_web_server.h"

static const char *TAG = "camera_web";

#define PART_BOUNDARY "123456789000000000000987654321"

static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t index_handler(httpd_req_t *req)
{
    const char *html =
        "<!DOCTYPE html>"
        "<html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<title>ESP32-S3 OV5640 Camera</title>"
        "<style>body{font-family:Arial, sans-serif;margin:24px;background:#f7f7f7;}"
        ".card{background:white;padding:18px;border-radius:12px;box-shadow:0 2px 10px #0001;}"
        "img{max-width:100%;border:1px solid #ddd;border-radius:8px;}"
        "a{display:inline-block;margin:8px 12px 8px 0;}</style>"
        "</head><body><div class='card'>"
        "<h2>ESP32-S3 OV5640 Camera Test</h2>"
        "<p>Use this page to verify whether OV5640 image capture works.</p>"
        "<p>"
        "<a href='/capture'>Capture one JPEG</a>"
        "<a href='/stream'>Open MJPEG stream</a>"
        "</p>"
        "<p>Power saving mode: this page does not auto-start camera stream.</p>"
        "<img src='/capture'>"
        "</div></body></html>";

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = camera_capture();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (fb->format != PIXFORMAT_JPEG) {
        ESP_LOGE(TAG, "Captured frame is not JPEG, format=%d", fb->format);
        camera_return(fb);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    camera_return(fb);
    return res;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char part_buf[64];

    while (true) {
        camera_fb_t *fb = camera_capture();
        if (!fb) {
            res = ESP_FAIL;
            break;
        }

        if (fb->format != PIXFORMAT_JPEG) {
            ESP_LOGE(TAG, "Captured frame is not JPEG, format=%d", fb->format);
            camera_return(fb);
            res = ESP_FAIL;
            break;
        }

        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);

        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }

        camera_return(fb);

        if (res != ESP_OK) {
            ESP_LOGI(TAG, "Stream client disconnected");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }

    return res;
}

void start_camera_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_uri_handlers = 8;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: 0x%x", err);
        return;
    }

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL,
    };

    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &capture_uri);
    httpd_register_uri_handler(server, &stream_uri);

    ESP_LOGI(TAG, "Camera web server started. Open http://192.168.4.1/ after connecting to ESP32S3_OV5640_AP");
}