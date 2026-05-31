#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "camera.h"
#include "camera_web_server.h"
#include "fruit_detect.h"

static const char *TAG = "camera_web";

#define PART_BOUNDARY "123456789000000000000987654321"

static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
static const char BASE64_TABLE[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

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

static esp_err_t send_detection_json(httpd_req_t *req, const fruit_detect_result_t *result)
{
    char buf[512];
    int len = snprintf(buf, sizeof(buf),
                       "{\"image_width\":%u,\"image_height\":%u,\"count\":%u,\"fruits\":[",
                       result->image_width, result->image_height, result->count);
    esp_err_t err = httpd_resp_send_chunk(req, buf, len);
    if (err != ESP_OK) {
        return err;
    }

    for (uint8_t i = 0; i < result->count; i++) {
        const fruit_info_t *f = &result->fruits[i];
        len = snprintf(buf, sizeof(buf),
                       "%s{\"center_x\":%u,\"center_y\":%u,\"diameter_px\":%u,"
                       "\"bbox_x\":%u,\"bbox_y\":%u,\"bbox_w\":%u,\"bbox_h\":%u,"
                       "\"area_px\":%lu,\"size_grade\":%u,\"grade_label\":\"%s\"}",
                       i == 0 ? "" : ",",
                       f->center_x, f->center_y, f->diameter_px,
                       f->bbox_x, f->bbox_y, f->bbox_w, f->bbox_h,
                       (unsigned long)f->area_px,
                       (unsigned int)f->size_grade,
                       fruit_grade_label(f->size_grade));
        err = httpd_resp_send_chunk(req, buf, len);
        if (err != ESP_OK) {
            return err;
        }
    }

    return send_text_chunk(req, "]}");
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    camera_fb_t *fb = camera_capture();
    if (!fb) {
        return httpd_resp_send(req,
                               "<!DOCTYPE html><html><body><h2>Capture failed</h2>"
                               "<p>Camera did not return a frame.</p></body></html>",
                               HTTPD_RESP_USE_STRLEN);
    }

    if (fb->format != PIXFORMAT_JPEG) {
        ESP_LOGE(TAG, "Captured frame is not JPEG, format=%d", fb->format);
        camera_return(fb);
        return httpd_resp_send(req,
                               "<!DOCTYPE html><html><body><h2>Capture failed</h2>"
                               "<p>Captured frame is not JPEG.</p></body></html>",
                               HTTPD_RESP_USE_STRLEN);
    }

    fruit_detect_result_t result;
    esp_err_t det_ret = fruit_detect_process(fb, &result);
    if (det_ret != ESP_OK) {
        memset(&result, 0, sizeof(result));
        result.image_width = fb->width;
        result.image_height = fb->height;
        ESP_LOGW(TAG, "Fruit detection failed: 0x%x", det_ret);
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
        ".ok{color:#55d68b}.warn{color:#ffbd5a}.small{color:#7bdff2}.medium{color:#ffbd5a}.large{color:#ff6b6b}"
        "</style></head><body><div class='bar'><h2>Citrus Sorter Debug</h2>"
        "<div class='actions'><a class='btn' href='/'>Refresh Capture</a>"
        "<a class='btn secondary' href='/capture'>Raw JPEG</a>"
        "<a class='btn secondary' href='/stream'>MJPEG Stream</a></div></div>"
        "<div class='wrap'><div class='stage'><canvas id='view'></canvas></div>"
        "<div class='cards'><div class='card'><div class='label'>Fruits Found</div><div class='value' id='count'>0</div></div>"
        "<div class='card'><div class='label'>Image Size</div><div class='value' id='size'>--</div></div>"
        "<div class='card'><div class='label'>Detection</div><div class='value' id='status'>--</div></div></div>"
        "<div id='table'></div><script>const detectOk=";

    res = send_text_chunk(req, page_start);
    if (res == ESP_OK) {
        res = send_text_chunk(req, det_ret == ESP_OK ? "true" : "false");
    }
    if (res == ESP_OK) {
        res = send_text_chunk(req, ";const data=");
    }
    if (res == ESP_OK) {
        res = send_detection_json(req, &result);
    }
    if (res == ESP_OK) {
        res = send_text_chunk(req, ";const imageSrc='data:image/jpeg;base64,");
    }
    if (res == ESP_OK) {
        res = send_base64_data(req, fb->buf, fb->len);
    }

    const char *page_end =
        "';const cv=document.getElementById('view');const ctx=cv.getContext('2d');"
        "const img=new Image();img.onload=function(){cv.width=img.naturalWidth;cv.height=img.naturalHeight;"
        "ctx.drawImage(img,0,0);drawOverlay();fillInfo();};img.src=imageSrc;"
        "function gradeClass(g){return g===2?'large':g===1?'medium':'small'}"
        "function drawOverlay(){for(let i=0;i<data.fruits.length;i++){const f=data.fruits[i];"
        "const cls=gradeClass(f.size_grade);const color=cls==='large'?'#ff6b6b':cls==='medium'?'#ffbd5a':'#7bdff2';"
        "ctx.strokeStyle=color;ctx.lineWidth=2;ctx.strokeRect(f.bbox_x,f.bbox_y,f.bbox_w,f.bbox_h);"
        "ctx.beginPath();ctx.arc(f.center_x,f.center_y,Math.max(4,f.diameter_px/2),0,Math.PI*2);ctx.stroke();"
        "ctx.beginPath();ctx.moveTo(f.center_x-5,f.center_y);ctx.lineTo(f.center_x+5,f.center_y);"
        "ctx.moveTo(f.center_x,f.center_y-5);ctx.lineTo(f.center_x,f.center_y+5);ctx.stroke();"
        "const label='#'+(i+1)+' '+f.grade_label+' D='+f.diameter_px+'px';ctx.font='bold 12px Arial';"
        "const w=ctx.measureText(label).width+8;let y=f.bbox_y-6;if(y<14)y=f.bbox_y+f.bbox_h+16;"
        "ctx.fillStyle=color;ctx.fillRect(f.bbox_x,y-13,w,16);ctx.fillStyle='#101418';ctx.fillText(label,f.bbox_x+4,y);}}"
        "function fillInfo(){document.getElementById('count').textContent=data.count;"
        "document.getElementById('size').textContent=data.image_width+' x '+data.image_height;"
        "const s=document.getElementById('status');s.textContent=detectOk?'OK':'Decode Error';s.className='value '+(detectOk?'ok':'warn');"
        "let html='';if(data.fruits.length===0){html='<div class=\"empty\">No citrus-colored fruit region found.</div>';}else{"
        "html='<table><thead><tr><th>#</th><th>Center</th><th>Diameter</th><th>Area</th><th>Box</th><th>Grade</th></tr></thead><tbody>';"
        "for(let i=0;i<data.fruits.length;i++){const f=data.fruits[i];const cls=gradeClass(f.size_grade);"
        "html+='<tr><td>'+(i+1)+'</td><td>'+f.center_x+', '+f.center_y+'</td><td>'+f.diameter_px+' px</td><td>'+f.area_px+' px</td><td>'+f.bbox_w+' x '+f.bbox_h+'</td><td class=\"'+cls+'\">'+f.grade_label+'</td></tr>';}"
        "html+='</tbody></table>';}document.getElementById('table').innerHTML=html;}"
        "</script></div></body></html>";

    if (res == ESP_OK) {
        res = send_text_chunk(req, page_end);
    }

    camera_return(fb);

    if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, NULL, 0);
    }
    return res;
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
