/*
 * Freenove ESP32-S3-WROOM CAM (FNK0085) as wireless camera for the JC8012P4A1C_I_W_Y.
 *
 * Opens its own Wi-Fi access point and serves the camera as MJPEG over HTTP:
 *   http://192.168.4.1/        small page with the live picture
 *   http://192.168.4.1/stream  multipart/x-mixed-replace MJPEG stream
 *   http://192.168.4.1/jpg     single JPEG picture
 *
 * The board's GC0308 sensor has no JPEG output, so an encoder task on core 1
 * compresses YUV422 frames with esp_new_jpeg into three JPEG buffers (triple
 * buffering) while the HTTP server sends the newest finished one.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_camera.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_jpeg_enc.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "cam";

/* Camera pins of the Freenove ESP32-S3-WROOM CAM (same as ESP32-S3-EYE) */
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 15
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 5
#define CAM_PIN_D0 11
#define CAM_PIN_D1 9
#define CAM_PIN_D2 8
#define CAM_PIN_D3 10
#define CAM_PIN_D4 12
#define CAM_PIN_D5 18
#define CAM_PIN_D6 17
#define CAM_PIN_D7 16
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_PCLK 13

#if CONFIG_CAM_FRAMESIZE_QVGA
#define CAM_FRAMESIZE FRAMESIZE_QVGA
#define CAM_W 320
#define CAM_H 240
#else
#define CAM_FRAMESIZE FRAMESIZE_VGA
#define CAM_W 640
#define CAM_H 480
#endif

#define JPEG_SLOTS 3
#define JPEG_SLOT_SIZE (CAM_W * CAM_H) /* generous: a JPEG at quality 60 is ~10x smaller */

#define PART_BOUNDARY "jc8012camframe"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

/* Triple-buffered JPEG frames shared between the encoder and the HTTP handlers */
static uint8_t *s_slot[JPEG_SLOTS];
static int s_slot_len[JPEG_SLOTS];
static int s_latest = -1;             /* newest finished slot */
static int s_in_use[JPEG_SLOTS];      /* number of senders reading the slot */
static uint32_t s_seq;                /* increments with every finished frame */
static SemaphoreHandle_t s_lock;
static volatile float s_encode_fps;

static esp_err_t camera_init(void)
{
    camera_config_t cfg = {
        .pin_pwdn = CAM_PIN_PWDN,
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
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_YUV422,
        .frame_size = CAM_FRAMESIZE,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        return err;
    }
    sensor_t *s = esp_camera_sensor_get();
    camera_sensor_info_t *info = esp_camera_sensor_get_info(&s->id);
    ESP_LOGI(TAG, "Camera sensor: %s, %dx%d YUV422", info ? info->name : "unknown", CAM_W, CAM_H);
    return ESP_OK;
}

static int pick_free_slot(void)
{
    for (int i = 0; i < JPEG_SLOTS; i++) {
        if (i != s_latest && s_in_use[i] == 0) {
            return i;
        }
    }
    return -1; /* cannot happen with 3 slots and one sender, but be safe */
}

static void encoder_task(void *arg)
{
    jpeg_enc_config_t cfg = DEFAULT_JPEG_ENC_CONFIG();
    cfg.width = CAM_W;
    cfg.height = CAM_H;
    cfg.src_type = JPEG_PIXEL_FORMAT_YCbYCr;
    cfg.subsampling = JPEG_SUBSAMPLE_420;
    cfg.quality = CONFIG_CAM_JPEG_QUALITY;
    jpeg_enc_handle_t enc = NULL;
    if (jpeg_enc_open(&cfg, &enc) != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "jpeg_enc_open failed");
        vTaskDelete(NULL);
    }
    /* The S3 encoder needs a 16-byte aligned input buffer. */
    const int yuv_len = CAM_W * CAM_H * 2;
    uint8_t *aligned_in = NULL;

    int frames = 0;
    int64_t t0 = esp_timer_get_time();
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        const uint8_t *in = fb->buf;
        if (((uintptr_t)in & 15) != 0) {
            if (!aligned_in) {
                aligned_in = jpeg_calloc_align(yuv_len, 16);
                ESP_LOGI(TAG, "Camera buffer not 16-byte aligned, copying frames");
            }
            memcpy(aligned_in, fb->buf, yuv_len);
            in = aligned_in;
        }

        xSemaphoreTake(s_lock, portMAX_DELAY);
        int slot = pick_free_slot();
        xSemaphoreGive(s_lock);

        int out_len = 0;
        jpeg_error_t err = JPEG_ERR_FAIL;
        if (slot >= 0) {
            err = jpeg_enc_process(enc, in, yuv_len, s_slot[slot], JPEG_SLOT_SIZE, &out_len);
        }
        esp_camera_fb_return(fb);
        if (err != JPEG_ERR_OK) {
            ESP_LOGW(TAG, "JPEG encode failed (%d)", err);
            continue;
        }

        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_slot_len[slot] = out_len;
        s_latest = slot;
        s_seq++;
        xSemaphoreGive(s_lock);

        frames++;
        int64_t now = esp_timer_get_time();
        if (now - t0 >= 5000000) {
            s_encode_fps = frames * 1e6f / (now - t0);
            ESP_LOGI(TAG, "Encoder: %.1f fps, %d KB/frame", s_encode_fps, out_len / 1024);
            frames = 0;
            t0 = now;
        }
    }
}

/* Borrow the newest frame newer than *seq; returns the slot or -1. */
static int acquire_frame(uint32_t *seq, int timeout_ms)
{
    for (int waited = 0; waited <= timeout_ms; waited += 5) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (s_latest >= 0 && s_seq != *seq) {
            int slot = s_latest;
            s_in_use[slot]++;
            *seq = s_seq;
            xSemaphoreGive(s_lock);
            return slot;
        }
        xSemaphoreGive(s_lock);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return -1;
}

static void release_frame(int slot)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_in_use[slot]--;
    xSemaphoreGive(s_lock);
}

static esp_err_t index_handler(httpd_req_t *req)
{
    static const char page[] =
        "<!doctype html><html><head><meta name=viewport content='width=device-width'>"
        "<title>JC8012 camera</title></head><body style='margin:0;background:#000'>"
        "<img src='/stream' style='width:100%'></body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, sizeof(page) - 1);
}

static esp_err_t jpg_handler(httpd_req_t *req)
{
    uint32_t seq = 0;
    int slot = acquire_frame(&seq, 1000);
    if (slot < 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    esp_err_t res = httpd_resp_send(req, (const char *)s_slot[slot], s_slot_len[slot]);
    release_frame(slot);
    return res;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    char part[80];
    uint32_t seq = 0;
    int frames = 0;
    size_t bytes = 0;
    int64_t t0 = esp_timer_get_time();
    ESP_LOGI(TAG, "Stream client connected");

    while (res == ESP_OK) {
        int slot = acquire_frame(&seq, 2000);
        if (slot < 0) {
            ESP_LOGW(TAG, "No new frame from the encoder");
            continue;
        }
        int len = s_slot_len[slot];
        int hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned)len);
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, part, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)s_slot[slot], len);
        }
        release_frame(slot);
        bytes += len;
        frames++;

        int64_t now = esp_timer_get_time();
        if (now - t0 >= 5000000) {
            float s = (now - t0) / 1e6f;
            ESP_LOGI(TAG, "Stream: %.1f fps sent, %.0f kbit/s", frames / s, bytes * 8 / s / 1000);
            frames = 0;
            bytes = 0;
            t0 = now;
        }
    }
    ESP_LOGI(TAG, "Stream client gone");
    return res;
}

static void start_http_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 6144;
    cfg.max_open_sockets = 4;
    cfg.lru_purge_enable = true;
    cfg.core_id = 0;
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &cfg));
    static const httpd_uri_t uris[] = {
        {.uri = "/", .method = HTTP_GET, .handler = index_handler},
        {.uri = "/jpg", .method = HTTP_GET, .handler = jpg_handler},
        {.uri = "/stream", .method = HTTP_GET, .handler = stream_handler},
    };
    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(server, &uris[i]);
    }
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "Client " MACSTR " joined", MAC2STR(e->mac));
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGI(TAG, "Client " MACSTR " left", MAC2STR(e->mac));
    }
}

static void wifi_start_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));

    wifi_config_t ap = {
        .ap = {
            .ssid = CONFIG_CAM_AP_SSID,
            .ssid_len = strlen(CONFIG_CAM_AP_SSID),
            .password = CONFIG_CAM_AP_PASSWORD,
            .channel = CONFIG_CAM_AP_CHANNEL,
            .max_connection = 3,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE); /* lowest latency for the video stream */
    ESP_LOGI(TAG, "Wi-Fi AP \"%s\" on channel %d, stream at http://192.168.4.1/stream", CONFIG_CAM_AP_SSID,
             CONFIG_CAM_AP_CHANNEL);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = camera_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
        return;
    }
    s_lock = xSemaphoreCreateMutex();
    for (int i = 0; i < JPEG_SLOTS; i++) {
        s_slot[i] = heap_caps_malloc(JPEG_SLOT_SIZE, MALLOC_CAP_SPIRAM);
        assert(s_slot[i]);
    }
    xTaskCreatePinnedToCore(encoder_task, "jpeg_enc", 6144, NULL, 5, NULL, 1);

    wifi_start_ap();
    start_http_server();
}
