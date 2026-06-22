#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "camera.h"
#include "camera_web_server.h"
#include "fruit_detect.h"
#include "m0_uart.h"

static const char *TAG = "camera_web";

#define PART_BOUNDARY "123456789000000000000987654321"
#define DETECTION_MAX_CAPTURE_ATTEMPTS 3
#define DETECTION_TASK_INTERVAL_MS 3000
#define DETECTION_TASK_STACK_SIZE 8192
#define DETECTION_TASK_PRIORITY 5

static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
static const char BASE64_TABLE[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

typedef struct {
    bool ready;
    bool detect_ok;
    fruit_detect_result_t result;
    m0_uart_payload_t payload;
    uint8_t *jpeg_data;
    size_t jpeg_len;
    uint32_t sequence;
} detection_cache_t;

typedef struct {
    bool detect_ok;
    fruit_detect_result_t result;
    m0_uart_payload_t payload;
    uint8_t *jpeg_data;
    size_t jpeg_len;
    uint32_t sequence;
} detection_snapshot_t;

static SemaphoreHandle_t s_detection_cache_lock;
static TaskHandle_t s_detection_task_handle;
static detection_cache_t s_detection_cache;

static void *cache_malloc(size_t size)
{
    void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ptr) {
        ptr = malloc(size);
    }
    return ptr;
}

static void release_detection_snapshot(detection_snapshot_t *snapshot)
{
    if (!snapshot) {
        return;
    }
    free(snapshot->jpeg_data);
    memset(snapshot, 0, sizeof(*snapshot));
}

static bool copy_latest_detection_snapshot(detection_snapshot_t *snapshot)
{
    if (!snapshot || !s_detection_cache_lock) {
        return false;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    if (xSemaphoreTake(s_detection_cache_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return false;
    }

    if (!s_detection_cache.ready || !s_detection_cache.jpeg_data ||
        s_detection_cache.jpeg_len == 0) {
        xSemaphoreGive(s_detection_cache_lock);
        return false;
    }

    snapshot->jpeg_data = cache_malloc(s_detection_cache.jpeg_len);
    if (!snapshot->jpeg_data) {
        xSemaphoreGive(s_detection_cache_lock);
        return false;
    }

    memcpy(snapshot->jpeg_data,
           s_detection_cache.jpeg_data,
           s_detection_cache.jpeg_len);
    snapshot->jpeg_len = s_detection_cache.jpeg_len;
    snapshot->detect_ok = s_detection_cache.detect_ok;
    snapshot->result = s_detection_cache.result;
    snapshot->payload = s_detection_cache.payload;
    snapshot->sequence = s_detection_cache.sequence;
    xSemaphoreGive(s_detection_cache_lock);
    return true;
}

static bool wait_for_latest_detection_snapshot(detection_snapshot_t *snapshot)
{
    for (uint8_t attempt = 0; attempt < 50; attempt++) {
        if (copy_latest_detection_snapshot(snapshot)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return false;
}

static bool update_detection_cache(const camera_fb_t *fb,
                                   const fruit_detect_result_t *result,
                                   const m0_uart_payload_t *payload,
                                   bool detect_ok)
{
    if (!fb || !result || !payload || !s_detection_cache_lock ||
        !fb->buf || fb->len == 0) {
        return false;
    }

    uint8_t *jpeg_copy = cache_malloc(fb->len);
    if (!jpeg_copy) {
        ESP_LOGE(TAG, "No memory for cached JPEG, need %u bytes",
                 (unsigned int)fb->len);
        return false;
    }
    memcpy(jpeg_copy, fb->buf, fb->len);

    if (xSemaphoreTake(s_detection_cache_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
        free(jpeg_copy);
        return false;
    }

    uint8_t *old_jpeg = s_detection_cache.jpeg_data;
    s_detection_cache.jpeg_data = jpeg_copy;
    s_detection_cache.jpeg_len = fb->len;
    s_detection_cache.result = *result;
    s_detection_cache.payload = *payload;
    s_detection_cache.detect_ok = detect_ok;
    s_detection_cache.ready = true;
    s_detection_cache.sequence++;
    xSemaphoreGive(s_detection_cache_lock);

    free(old_jpeg);
    return true;
}

static esp_err_t send_text_chunk(httpd_req_t *req, const char *text)
{
    return httpd_resp_send_chunk(req, text, strlen(text));
}

static size_t base64_encode_block(const uint8_t *src, size_t len, char *dst)
{
    size_t out = 0;

    for (size_t i = 0; i < len; i += 3) {
        uint32_t triple = (uint32_t)src[i] << 16;
        bool have_b = i + 1 < len;
        bool have_c = i + 2 < len;

        if (have_b) {
            triple |= (uint32_t)src[i + 1] << 8;
        }
        if (have_c) {
            triple |= src[i + 2];
        }

        dst[out++] = BASE64_TABLE[(triple >> 18) & 0x3f];
        dst[out++] = BASE64_TABLE[(triple >> 12) & 0x3f];
        dst[out++] = have_b ? BASE64_TABLE[(triple >> 6) & 0x3f] : '=';
        dst[out++] = have_c ? BASE64_TABLE[triple & 0x3f] : '=';
    }

    return out;
}

static esp_err_t send_base64_data(httpd_req_t *req, const uint8_t *data, size_t len)
{
    char out[1024];
    size_t pos = 0;

    while (pos < len) {
        size_t chunk = len - pos;
        if (chunk > 768) {
            chunk = 768;
        }
        if (pos + chunk < len) {
            chunk -= chunk % 3;
        }

        size_t out_len = base64_encode_block(data + pos, chunk, out);
        esp_err_t err = httpd_resp_send_chunk(req, out, out_len);
        if (err != ESP_OK) {
            return err;
        }
        pos += chunk;
    }

    return ESP_OK;
}

static esp_err_t send_detection_json(httpd_req_t *req,
                                     const fruit_detect_result_t *result,
                                     const m0_uart_payload_t *payload,
                                     bool detect_ok,
                                     const uint8_t *image_data,
                                     size_t image_len)
{
    const char *reference_mode = "none";
    if (result->board.reference_mode == BOARD_REFERENCE_BLUE_DOTS) {
        reference_mode = "blue_dots";
    }

    char buf[768];
    int len = snprintf(buf, sizeof(buf),
                       "{\"detect_ok\":%s,\"image_width\":%u,\"image_height\":%u,\"count\":%u,"
                       "\"board\":{\"found\":%s,\"center_x\":%u,\"center_y\":%u,"
                       "\"reference_mode\":\"%s\","
                       "\"bbox_x\":%u,\"bbox_y\":%u,\"bbox_w\":%u,\"bbox_h\":%u,"
                       "\"tl_x\":%.2f,\"tl_y\":%.2f,\"tr_x\":%.2f,\"tr_y\":%.2f,"
                       "\"br_x\":%.2f,\"br_y\":%.2f,\"bl_x\":%.2f,\"bl_y\":%.2f,"
                       "\"area_px\":%lu},"
                       "\"uart\":{\"header\":\"0xAA\",\"has_fruit\":%u,\"grade\":%u,"
                       "\"x\":%.2f,\"y\":%.2f,\"relative_valid\":%s},"
                       "\"fruits\":[",
                       detect_ok ? "true" : "false",
                       result->image_width, result->image_height, result->count,
                       result->board.found ? "true" : "false",
                       result->board.center_x, result->board.center_y,
                       reference_mode,
                       result->board.bbox_x, result->board.bbox_y,
                       result->board.bbox_w, result->board.bbox_h,
                       result->board.tl_x, result->board.tl_y,
                       result->board.tr_x, result->board.tr_y,
                       result->board.br_x, result->board.br_y,
                       result->board.bl_x, result->board.bl_y,
                       (unsigned long)result->board.area_px,
                       payload->has_fruit, payload->grade,
                       payload->x, payload->y,
                       payload->world_valid ? "true" : "false");
    esp_err_t err = httpd_resp_send_chunk(req, buf, len);
    if (err != ESP_OK) {
        return err;
    }

    for (uint8_t i = 0; i < result->count; i++) {
        const fruit_info_t *f = &result->fruits[i];
        float relative_x = 0.0f;
        float relative_y = 0.0f;
        bool relative_valid = fruit_detect_board_relative_coord(&result->board,
                                                                f->center_x,
                                                                f->center_y,
                                                                &relative_x,
                                                                &relative_y);
        uint16_t x_min = f->bbox_x;
        uint16_t x_max = f->bbox_x + f->bbox_w - 1;
        uint16_t y_min = f->bbox_y;
        uint16_t y_max = f->bbox_y + f->bbox_h - 1;
        len = snprintf(buf, sizeof(buf),
                       "%s{\"center_x\":%u,\"center_y\":%u,\"diameter_px\":%u,"
                       "\"bbox_x\":%u,\"bbox_y\":%u,\"bbox_w\":%u,\"bbox_h\":%u,"
                       "\"x_min\":%u,\"x_max\":%u,\"y_min\":%u,\"y_max\":%u,"
                       "\"relative_valid\":%s,\"relative_x\":%.2f,\"relative_y\":%.2f,"
                       "\"area_px\":%lu,\"size_grade\":%u,\"grade_label\":\"%s\"}",
                       i == 0 ? "" : ",",
                       f->center_x, f->center_y, f->diameter_px,
                       f->bbox_x, f->bbox_y, f->bbox_w, f->bbox_h,
                       x_min, x_max, y_min, y_max,
                       relative_valid ? "true" : "false", relative_x, relative_y,
                       (unsigned long)f->area_px,
                       (unsigned int)f->size_grade,
                       fruit_grade_label(f->size_grade));
        err = httpd_resp_send_chunk(req, buf, len);
        if (err != ESP_OK) {
            return err;
        }
    }

    if (!image_data || image_len == 0) {
        return send_text_chunk(req, "]}");
    }

    err = send_text_chunk(req, "],\"image\":\"data:image/jpeg;base64,");
    if (err != ESP_OK) {
        return err;
    }
    err = send_base64_data(req, image_data, image_len);
    if (err != ESP_OK) {
        return err;
    }
    return send_text_chunk(req, "\"}");
}

static bool detection_result_ready_for_output(const fruit_detect_result_t *result)
{
    return result &&
           result->board.found &&
           fruit_detect_board_geometry_valid(&result->board);
}

static void clear_detection_for_output(fruit_detect_result_t *result,
                                       const camera_fb_t *fb)
{
    if (!result) {
        return;
    }

    uint16_t width = fb ? fb->width : result->image_width;
    uint16_t height = fb ? fb->height : result->image_height;
    memset(result, 0, sizeof(*result));
    result->image_width = width;
    result->image_height = height;
}

static esp_err_t capture_stable_detection(camera_fb_t **out_fb,
                                          fruit_detect_result_t *result,
                                          bool *detect_ok)
{
    if (!out_fb || !result || !detect_ok) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_fb = NULL;
    *detect_ok = false;
    memset(result, 0, sizeof(*result));

    for (uint8_t attempt = 0; attempt < DETECTION_MAX_CAPTURE_ATTEMPTS; attempt++) {
        camera_fb_t *fb = camera_capture();
        if (!fb) {
            return ESP_FAIL;
        }

        if (fb->format != PIXFORMAT_JPEG) {
            ESP_LOGE(TAG, "Captured frame is not JPEG, format=%d", fb->format);
            camera_return(fb);
            return ESP_FAIL;
        }

        fruit_detect_result_t candidate;
        esp_err_t ret = fruit_detect_process(fb, &candidate);
        if (ret == ESP_OK && detection_result_ready_for_output(&candidate)) {
            *out_fb = fb;
            *result = candidate;
            *detect_ok = true;
            return ESP_OK;
        }

        ESP_LOGW(TAG, "Rejected unstable detection frame %u/%u: ret=0x%x board=%u",
                 (unsigned int)(attempt + 1),
                 (unsigned int)DETECTION_MAX_CAPTURE_ATTEMPTS,
                 ret,
                 ret == ESP_OK && candidate.board.found ? 1U : 0U);

        if (attempt + 1 == DETECTION_MAX_CAPTURE_ATTEMPTS) {
            *out_fb = fb;
            if (ret == ESP_OK) {
                *result = candidate;
                clear_detection_for_output(result, fb);
            } else {
                clear_detection_for_output(result, fb);
            }
            *detect_ok = false;
            return ESP_OK;
        }

        camera_return(fb);
        vTaskDelay(pdMS_TO_TICKS(80));
    }

    return ESP_FAIL;
}

static void autonomous_detection_task(void *arg)
{
    (void)arg;

    while (true) {
        camera_fb_t *fb = NULL;
        fruit_detect_result_t result;
        bool detect_ok = false;
        esp_err_t ret = capture_stable_detection(&fb, &result, &detect_ok);

        if (ret == ESP_OK && fb) {
            m0_uart_payload_t payload = {0};
            if (detect_ok) {
                m0_uart_build_payload(&result, &payload);
                esp_err_t uart_ret = m0_uart_send_result(&result);
                if (uart_ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to send autonomous result to M0: 0x%x",
                             uart_ret);
                }
            } else {
                ESP_LOGW(TAG, "No stable autonomous detection; skipping M0 output");
            }

            if (!update_detection_cache(fb, &result, &payload, detect_ok)) {
                ESP_LOGW(TAG, "Failed to update latest detection cache");
            }
            camera_return(fb);
        } else {
            ESP_LOGW(TAG, "Autonomous capture failed: 0x%x", ret);
        }

        vTaskDelay(pdMS_TO_TICKS(DETECTION_TASK_INTERVAL_MS));
    }
}

esp_err_t start_camera_detection_task(void)
{
    if (s_detection_task_handle) {
        return ESP_OK;
    }

    if (!s_detection_cache_lock) {
        s_detection_cache_lock = xSemaphoreCreateMutex();
        if (!s_detection_cache_lock) {
            return ESP_ERR_NO_MEM;
        }
    }

    BaseType_t created = xTaskCreate(autonomous_detection_task,
                                     "camera_detect",
                                     DETECTION_TASK_STACK_SIZE,
                                     NULL,
                                     DETECTION_TASK_PRIORITY,
                                     &s_detection_task_handle);
    if (created != pdPASS) {
        s_detection_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Autonomous detection task started, interval=%d ms",
             DETECTION_TASK_INTERVAL_MS);
    return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    detection_snapshot_t snapshot;
    if (!wait_for_latest_detection_snapshot(&snapshot)) {
        return httpd_resp_send(req,
                               "<!DOCTYPE html><html><body><h2>Detection not ready</h2>"
                               "<p>The autonomous detector has not produced a frame yet.</p>"
                               "</body></html>",
                               HTTPD_RESP_USE_STRLEN);
    }

    esp_err_t res = ESP_OK;
    const char *page_start =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<title>Citrus Sorter Debug</title>"
        "<style>"
        "*{box-sizing:border-box}body{margin:0;font-family:Arial,sans-serif;background:#101418;color:#e9edf2}"
        ".bar{display:flex;align-items:center;justify-content:space-between;gap:12px;flex-wrap:wrap;padding:12px 16px;background:#17202a;border-bottom:1px solid #263645}"
        ".bar h2{margin:0;font-size:18px}.actions{display:flex;gap:8px;align-items:center;flex-wrap:wrap}"
        "a.btn{color:#fff;background:#2f7dd1;text-decoration:none;padding:7px 12px;border-radius:6px;font-weight:700;font-size:13px}"
        "a.btn.secondary{background:#304050}.wrap{padding:14px;max-width:980px;margin:0 auto}"
        ".stage{background:#050608;border:1px solid #263645;border-radius:8px;overflow:hidden;line-height:0}"
        "canvas{display:block;max-width:100%;height:auto;margin:0 auto}"
        ".cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin:12px 0}"
        ".card{background:#17202a;border:1px solid #263645;border-radius:8px;padding:12px}.label{font-size:12px;color:#9aa7b5}.value{font-size:24px;font-weight:800;margin-top:4px}"
        "table{width:100%;border-collapse:collapse;background:#17202a;border:1px solid #263645;border-radius:8px;overflow:hidden}"
        "th,td{padding:8px;border-bottom:1px solid #263645;text-align:left;font-size:13px}th{background:#223042;color:#b9c6d3}.empty{padding:14px;color:#9aa7b5}"
        ".ok{color:#55d68b}.warn{color:#ffbd5a}.small{color:#7bdff2}.large{color:#ff6b6b}"
        "</style></head><body><div class='bar'><h2>Citrus Sorter Debug</h2>"
        "<div class='actions'><a class='btn' href='/'>Refresh Latest Result</a>"
        "<a class='btn secondary' href='/?auto=1'>Auto Refresh (3s)</a>"
        "<a class='btn secondary' href='/capture'>Raw JPEG</a>"
        "<a class='btn secondary' href='/stream'>MJPEG Stream</a></div></div>"
        "<div class='wrap'><div class='stage'><canvas id='view'></canvas></div>"
        "<div class='cards'><div class='card'><div class='label'>Fruits Found</div><div class='value' id='count'>0</div></div>"
        "<div class='card'><div class='label'>Image Size</div><div class='value' id='size'>--</div></div>"
        "<div class='card'><div class='label'>Detection</div><div class='value' id='status'>--</div></div>"
        "<div class='card'><div class='label'>Reference</div><div class='value' id='boardstatus'>--</div></div>"
        "<div class='card'><div class='label'>M0 Frame</div><div class='value' id='m0frame'>--</div></div></div>"
        "<div class='card'><div class='label'>Reference Coordinates</div><div id='m0coords' style='font-size:14px;margin-top:6px;line-height:1.7'></div></div>"
        "<div id='table'></div><script>let data=";

    res = send_text_chunk(req, page_start);
    if (res == ESP_OK) {
        res = send_detection_json(req,
                                  &snapshot.result,
                                  &snapshot.payload,
                                  snapshot.detect_ok,
                                  NULL,
                                  0);
    }
    if (res == ESP_OK) {
        res = send_text_chunk(req, ";let detectOk=!!data.detect_ok;let imageSrc='data:image/jpeg;base64,");
    }
    if (res == ESP_OK) {
        res = send_base64_data(req, snapshot.jpeg_data, snapshot.jpeg_len);
    }

    bool auto_refresh = false;
    size_t query_len = httpd_req_get_url_query_len(req) + 1;
    char query[32];
    if (query_len > 1 && query_len <= sizeof(query) &&
        httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char auto_value[8];
        auto_refresh = httpd_query_key_value(query, "auto", auto_value, sizeof(auto_value)) == ESP_OK &&
                       strcmp(auto_value, "1") == 0;
    }

    const char *page_end =
        "';const cv=document.getElementById('view');const ctx=cv.getContext('2d');let refreshBusy=false;"
        "function loadImage(){const img=new Image();img.onload=function(){cv.width=img.naturalWidth;cv.height=img.naturalHeight;"
        "ctx.drawImage(img,0,0);drawOverlay();fillInfo();};img.src=imageSrc;}loadImage();"
        "function gradeClass(g){return g===1?'large':'small'}"
        "function drawOverlay(){const b=data.board;if(b.found){const refLabel='blue dots reference';ctx.strokeStyle='#c9894b';ctx.lineWidth=3;ctx.beginPath();"
        "ctx.moveTo(b.tl_x,b.tl_y);ctx.lineTo(b.tr_x,b.tr_y);ctx.lineTo(b.br_x,b.br_y);ctx.lineTo(b.bl_x,b.bl_y);ctx.closePath();ctx.stroke();"
        "ctx.font='bold 12px Arial';ctx.fillStyle='#c9894b';ctx.fillText(refLabel,b.tl_x+4,Math.max(12,b.tl_y-6));"
        "ctx.fillStyle='#00a8ff';for(const p of [[b.tl_x,b.tl_y],[b.tr_x,b.tr_y],[b.br_x,b.br_y],[b.bl_x,b.bl_y]]){ctx.beginPath();ctx.arc(p[0],p[1],5,0,Math.PI*2);ctx.fill();}}"
        "for(let i=0;i<data.fruits.length;i++){const f=data.fruits[i];"
        "const cls=gradeClass(f.size_grade);const color=cls==='large'?'#ff6b6b':'#7bdff2';"
        "ctx.strokeStyle=color;ctx.lineWidth=2;ctx.strokeRect(f.bbox_x,f.bbox_y,f.bbox_w,f.bbox_h);"
        "ctx.beginPath();ctx.arc(f.center_x,f.center_y,Math.max(4,f.diameter_px/2),0,Math.PI*2);ctx.stroke();"
        "ctx.beginPath();ctx.moveTo(f.center_x-5,f.center_y);ctx.lineTo(f.center_x+5,f.center_y);"
        "ctx.moveTo(f.center_x,f.center_y-5);ctx.lineTo(f.center_x,f.center_y+5);ctx.stroke();"
        "const label='#'+(i+1)+' '+f.grade_label+' D='+f.diameter_px+'px';ctx.font='bold 12px Arial';"
        "const w=ctx.measureText(label).width+8;let y=f.bbox_y-6;if(y<14)y=f.bbox_y+f.bbox_h+16;"
        "ctx.fillStyle=color;ctx.fillRect(f.bbox_x,y-13,w,16);ctx.fillStyle='#101418';ctx.fillText(label,f.bbox_x+4,y);}}"
        "function fillInfo(){document.getElementById('count').textContent=data.count;"
        "document.getElementById('size').textContent=data.image_width+' x '+data.image_height;"
        "const s=document.getElementById('status');s.textContent=detectOk?'OK':'No Stable Frame';s.className='value '+(detectOk?'ok':'warn');"
        "const b=data.board;document.getElementById('boardstatus').textContent=b.found?(b.reference_mode+' '+b.bbox_w+' x '+b.bbox_h):'--';"
        "const u=data.uart;document.getElementById('m0frame').textContent=u.header+' '+u.has_fruit+' '+u.grade+' '+u.x.toFixed(2)+' '+u.y.toFixed(2);"
        "document.getElementById('m0coords').innerHTML='has_fruit: '+u.has_fruit+' &nbsp; grade: '+u.grade+'<br>'"
        "+'sent x%: '+u.x.toFixed(2)+' &nbsp; sent y%: '+u.y.toFixed(2)+'<br>'"
        "+'mode: '+(b.found?b.reference_mode:'--')+'<br>'"
        "+'coordinate frame: TL=(0,0), TR=(100,0), BR=(100,100), BL=(0,100)<br>'"
        "+'image points: '+(b.found?('TL '+b.tl_x.toFixed(1)+','+b.tl_y.toFixed(1)+' TR '+b.tr_x.toFixed(1)+','+b.tr_y.toFixed(1)+'<br>BL '+b.bl_x.toFixed(1)+','+b.bl_y.toFixed(1)+' BR '+b.br_x.toFixed(1)+','+b.br_y.toFixed(1)):'--');"
        "let html='';if(data.fruits.length===0){html='<div class=\"empty\">No citrus-colored fruit region found.</div>';}else{"
        "html='<table><thead><tr><th>#</th><th>Pixel Center</th><th>Bounds</th><th>Reference Position (%)</th><th>Diameter</th><th>Area</th><th>Box</th><th>Grade</th></tr></thead><tbody>';"
        "for(let i=0;i<data.fruits.length;i++){const f=data.fruits[i];const cls=gradeClass(f.size_grade);"
        "const rel=f.relative_valid?(f.relative_x.toFixed(2)+', '+f.relative_y.toFixed(2)):'--';"
        "const bounds='x '+f.x_min+'..'+f.x_max+'<br>y '+f.y_min+'..'+f.y_max;"
        "html+='<tr><td>'+(i+1)+'</td><td>'+f.center_x+', '+f.center_y+'</td><td>'+bounds+'</td><td>'+rel+'</td><td>'+f.diameter_px+' px</td><td>'+f.area_px+' px</td><td>'+f.bbox_w+' x '+f.bbox_h+'</td><td class=\"'+cls+'\">'+f.grade_label+'</td></tr>';}"
        "html+='</tbody></table>';}document.getElementById('table').innerHTML=html;}"
        "async function refreshSnapshot(){if(refreshBusy)return;refreshBusy=true;try{"
        "const r=await fetch('/snapshot?t='+Date.now(),{cache:'no-store'});if(!r.ok)throw new Error('snapshot');"
        "const next=await r.json();data=next;detectOk=!!next.detect_ok;if(next.image){imageSrc=next.image;loadImage();}else{fillInfo();}"
        "}catch(e){const s=document.getElementById('status');s.textContent='Refresh Error';s.className='value warn';}"
        "finally{refreshBusy=false;}}"
        "const autoRefresh=";

    if (res == ESP_OK) {
        res = send_text_chunk(req, page_end);
    }
    if (res == ESP_OK) {
        res = send_text_chunk(req, auto_refresh ? "true" : "false");
    }

    const char *page_finish =
        ";if(autoRefresh){setInterval(refreshSnapshot,3000);}"
        "</script></div></body></html>";

    if (res == ESP_OK) {
        res = send_text_chunk(req, page_finish);
    }

    release_detection_snapshot(&snapshot);

    if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, NULL, 0);
    }
    return res;
}

static esp_err_t snapshot_handler(httpd_req_t *req)
{
    detection_snapshot_t snapshot;
    if (!copy_latest_detection_snapshot(&snapshot)) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t res = send_detection_json(req,
                                         &snapshot.result,
                                         &snapshot.payload,
                                         snapshot.detect_ok,
                                         snapshot.jpeg_data,
                                         snapshot.jpeg_len);
    release_detection_snapshot(&snapshot);
    if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, NULL, 0);
    }
    return res;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    detection_snapshot_t snapshot;
    if (!copy_latest_detection_snapshot(&snapshot)) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t res = httpd_resp_send(req,
                                    (const char *)snapshot.jpeg_data,
                                    snapshot.jpeg_len);
    release_detection_snapshot(&snapshot);
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
    uint32_t last_sequence = 0;

    while (true) {
        detection_snapshot_t snapshot;
        if (!copy_latest_detection_snapshot(&snapshot)) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (snapshot.sequence == last_sequence) {
            release_detection_snapshot(&snapshot);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        last_sequence = snapshot.sequence;

        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, snapshot.jpeg_len);

        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req,
                                        (const char *)snapshot.jpeg_data,
                                        snapshot.jpeg_len);
        }

        release_detection_snapshot(&snapshot);

        if (res != ESP_OK) {
            ESP_LOGI(TAG, "Stream client disconnected");
            break;
        }
    }

    return res;
}

void start_camera_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_uri_handlers = 8;
    config.stack_size = 8192;

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

    httpd_uri_t snapshot_uri = {
        .uri = "/snapshot",
        .method = HTTP_GET,
        .handler = snapshot_handler,
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
    httpd_register_uri_handler(server, &snapshot_uri);
    httpd_register_uri_handler(server, &stream_uri);

    ESP_LOGI(TAG, "Camera web server started. Open http://192.168.4.1/ after connecting to ESP32S3_OV5640_AP");
}
