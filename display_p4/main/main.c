/*
 * Wireless camera display for the Guition JC8012P4A1C_I_W_Y (ESP32-P4).
 *
 * Joins the Wi-Fi of the Freenove camera through the on-board ESP32-C6 (ESP-Hosted),
 * reads its MJPEG stream over HTTP, decodes the frames with the hardware JPEG decoder
 * and shows them in landscape (1280x800) via PPA scale + rotate.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/jpeg_decode.h"
#include "driver/ppa.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_hosted.h"
#include "esp_http_client.h"
#include "esp_lcd_jd9365.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

extern const jd9365_lcd_init_cmd_t jc8012_lcd_init_cmds[];
extern const size_t jc8012_lcd_init_cmds_count;

static const char *TAG = "wcam";

/* Panel (native portrait) and landscape view */
#define PANEL_W 800
#define PANEL_H 1280
#define VIEW_W 1280
#define VIEW_H 800
#define PANEL_FB_BYTES (PANEL_W * PANEL_H * 2)
#define LCD_BACKLIGHT GPIO_NUM_23
#define LCD_RESET GPIO_NUM_27

#define MAX_FRAME_W 1280
#define MAX_FRAME_H 960
#define JPEG_IN_SIZE (512 * 1024)

static esp_lcd_panel_handle_t s_panel;
static void *s_fb[2];
static int s_back;
static SemaphoreHandle_t s_vsync;
static ppa_client_handle_t s_ppa;
static jpeg_decoder_handle_t s_jpeg;
static uint8_t *s_jpeg_in;
static size_t s_jpeg_in_size;
static uint8_t *s_jpeg_out;
static size_t s_jpeg_out_size;
static EventGroupHandle_t s_net;
#define NET_GOT_IP BIT0

/* ---------------------------------------------------------------- display */

static bool IRAM_ATTR on_frame_done(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_vsync, &woken);
    return woken == pdTRUE;
}

static esp_err_t display_init(void)
{
    esp_ldo_channel_handle_t ldo = NULL;
    esp_ldo_channel_config_t ldo_cfg = {.chan_id = 3, .voltage_mv = 2500};
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_cfg, &ldo), TAG, "DSI PHY LDO");

    esp_lcd_dsi_bus_handle_t bus = NULL;
    esp_lcd_dsi_bus_config_t bus_cfg = {.bus_id = 0, .num_data_lanes = 2, .lane_bit_rate_mbps = 1500};
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_cfg, &bus), TAG, "DSI bus");

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_dbi_io_config_t io_cfg = {.virtual_channel = 0, .lcd_cmd_bits = 8, .lcd_param_bits = 8};
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_dbi(bus, &io_cfg, &io), TAG, "DBI IO");

    esp_lcd_dpi_panel_config_t dpi_cfg = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 70,
        .in_color_format = LCD_COLOR_FMT_RGB565,
        .out_color_format = LCD_COLOR_FMT_RGB565,
        .num_fbs = 2,
        .video_timing = {
            .h_size = PANEL_W, .v_size = PANEL_H,
            .hsync_pulse_width = 20, .hsync_back_porch = 20, .hsync_front_porch = 40,
            .vsync_pulse_width = 4, .vsync_back_porch = 10, .vsync_front_porch = 30,
        },
    };
    jd9365_vendor_config_t vendor_cfg = {
        .init_cmds = jc8012_lcd_init_cmds,
        .init_cmds_size = (uint16_t)jc8012_lcd_init_cmds_count,
        .mipi_config = {.dsi_bus = bus, .dpi_config = &dpi_cfg, .lane_num = 2},
    };
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_RESET,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_cfg,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_jd9365(io, &panel_cfg, &s_panel), TAG, "JD9365");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "disp on");

    ESP_RETURN_ON_ERROR(esp_lcd_dpi_panel_get_frame_buffer(s_panel, 2, &s_fb[0], &s_fb[1]), TAG, "fb");
    for (int i = 0; i < 2; i++) {
        memset(s_fb[i], 0, PANEL_FB_BYTES);
        esp_lcd_panel_draw_bitmap(s_panel, 0, 0, PANEL_W, PANEL_H, s_fb[i]);
    }
    s_back = 0;

    s_vsync = xSemaphoreCreateBinary();
    esp_lcd_dpi_panel_event_callbacks_t cbs = {.on_frame_buf_complete = on_frame_done};
    ESP_RETURN_ON_ERROR(esp_lcd_dpi_panel_register_event_callbacks(s_panel, &cbs, NULL), TAG, "cb");

    gpio_config_t bl = {.pin_bit_mask = 1ULL << LCD_BACKLIGHT, .mode = GPIO_MODE_OUTPUT};
    gpio_config(&bl);
    gpio_set_level(LCD_BACKLIGHT, 1);

    ppa_client_config_t ppa_cfg = {.oper_type = PPA_OPERATION_SRM, .max_pending_trans_num = 1};
    ESP_RETURN_ON_ERROR(ppa_register_client(&ppa_cfg, &s_ppa), TAG, "PPA");
    return ESP_OK;
}

static void display_clear(void)
{
    for (int i = 0; i < 2; i++) {
        memset(s_fb[i], 0, PANEL_FB_BYTES);
        esp_lcd_panel_draw_bitmap(s_panel, 0, 0, PANEL_W, PANEL_H, s_fb[i]);
    }
    s_back = 0;
}

/* Scale the picture to fit 1280x800, rotate it into the portrait panel buffer
 * (centered, black borders) and flip buffers. */
static esp_err_t display_show(const void *rgb565, uint32_t w, uint32_t h)
{
    float s = (float)VIEW_W / w;
    if ((float)VIEW_H / h < s) {
        s = (float)VIEW_H / h;
    }
    s = (float)(int)(s * 16.0f) / 16.0f; /* PPA uses 1/16 steps */
    uint32_t out_w = (uint32_t)(w * s);
    uint32_t out_h = (uint32_t)(h * s);

    ppa_srm_oper_config_t op = {
        .in = {
            .buffer = rgb565, .pic_w = w, .pic_h = h,
            .block_w = w, .block_h = h, .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = s_fb[s_back], .buffer_size = PANEL_FB_BYTES,
            .pic_w = PANEL_W, .pic_h = PANEL_H,
            .block_offset_x = (PANEL_W - out_h) / 2,
            .block_offset_y = (PANEL_H - out_w) / 2,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_270,
        .scale_x = s,
        .scale_y = s,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    static bool flip_pending;
    if (flip_pending) {
        xSemaphoreTake(s_vsync, pdMS_TO_TICKS(100)); /* back buffer no longer scanned out */
    }
    ESP_RETURN_ON_ERROR(ppa_do_scale_rotate_mirror(s_ppa, &op), TAG, "PPA SRM");
    xSemaphoreTake(s_vsync, 0);
    ESP_RETURN_ON_ERROR(esp_lcd_panel_draw_bitmap(s_panel, 0, 0, PANEL_W, PANEL_H, s_fb[s_back]), TAG, "flip");
    flip_pending = true;
    s_back ^= 1;
    return ESP_OK;
}

/* ------------------------------------------------------------------- Wi-Fi */

static void net_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_net, NET_GOT_IP);
        ESP_LOGW(TAG, "Wi-Fi \"%s\" not reachable, retrying", CONFIG_CAM_AP_SSID);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        ESP_LOGI(TAG, "Connected to \"%s\", IP " IPSTR, CONFIG_CAM_AP_SSID, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_net, NET_GOT_IP);
    }
}

static void wifi_init(void)
{
    s_net = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, net_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, net_event, NULL));

    esp_hosted_coprocessor_fwver_t ver = {0};
    if (esp_hosted_get_coprocessor_fwversion(&ver) == ESP_OK) {
        ESP_LOGI(TAG, "C6 runs ESP-Hosted %" PRIu32 ".%" PRIu32 ".%" PRIu32, ver.major1, ver.minor1, ver.patch1);
    }

    wifi_config_t sta = {0};
    strlcpy((char *)sta.sta.ssid, CONFIG_CAM_AP_SSID, sizeof(sta.sta.ssid));
    strlcpy((char *)sta.sta.password, CONFIG_CAM_AP_PASSWORD, sizeof(sta.sta.password));
    sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);
}

/* ------------------------------------------------------------ MJPEG stream */

/* Small buffered reader on top of esp_http_client_read() */
typedef struct {
    esp_http_client_handle_t client;
    uint8_t buf[2048];
    int pos;
    int len;
} reader_t;

static int reader_fill(reader_t *r)
{
    r->pos = 0;
    r->len = esp_http_client_read(r->client, (char *)r->buf, sizeof(r->buf));
    return r->len;
}

/* Reads one header line (without CR/LF). Returns its length or -1. */
static int read_line(reader_t *r, char *line, int max)
{
    int n = 0;
    while (1) {
        if (r->pos >= r->len && reader_fill(r) <= 0) {
            return -1;
        }
        char c = r->buf[r->pos++];
        if (c == '\n') {
            if (n > 0 && line[n - 1] == '\r') {
                n--;
            }
            line[n] = 0;
            return n;
        }
        if (n < max - 1) {
            line[n++] = c;
        }
    }
}

static int read_exact(reader_t *r, uint8_t *dst, int count)
{
    int done = 0;
    int avail = r->len - r->pos;
    if (avail > 0) {
        int take = avail < count ? avail : count;
        memcpy(dst, r->buf + r->pos, take);
        r->pos += take;
        done = take;
    }
    while (done < count) {
        int got = esp_http_client_read(r->client, (char *)dst + done, count - done);
        if (got <= 0) {
            return -1;
        }
        done += got;
    }
    return done;
}

static void show_jpeg(int len)
{
    static uint32_t last_w, last_h;
    jpeg_decode_picture_info_t info;
    if (jpeg_decoder_get_info(s_jpeg_in, len, &info) != ESP_OK || info.width > MAX_FRAME_W ||
        info.height > MAX_FRAME_H) {
        ESP_LOGW(TAG, "Bad JPEG frame (%d bytes)", len);
        return;
    }
    jpeg_decode_cfg_t dec_cfg = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
        .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
    };
    uint32_t out_size = 0;
    if (jpeg_decoder_process(s_jpeg, &dec_cfg, s_jpeg_in, len, s_jpeg_out, s_jpeg_out_size, &out_size) != ESP_OK) {
        ESP_LOGW(TAG, "JPEG decode failed");
        return;
    }
    if (info.width != last_w || info.height != last_h) {
        display_clear();
        last_w = info.width;
        last_h = info.height;
        ESP_LOGI(TAG, "Showing %" PRIu32 "x%" PRIu32 " MJPEG", last_w, last_h);
    }
    display_show(s_jpeg_out, info.width, info.height);
}

static void stream_task(void *arg)
{
    char line[128];
    reader_t *r = calloc(1, sizeof(reader_t));
    while (1) {
        xEventGroupWaitBits(s_net, NET_GOT_IP, pdFALSE, pdTRUE, portMAX_DELAY);
        esp_http_client_config_t cfg = {
            .url = CONFIG_CAM_STREAM_URL,
            .timeout_ms = 5000,
            .buffer_size = 4096,
        };
        esp_http_client_handle_t client = esp_http_client_init(&cfg);
        if (esp_http_client_open(client, 0) != ESP_OK) {
            ESP_LOGW(TAG, "Cannot open %s", CONFIG_CAM_STREAM_URL);
            esp_http_client_cleanup(client);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        esp_http_client_fetch_headers(client);
        ESP_LOGI(TAG, "Streaming from %s (HTTP %d)", CONFIG_CAM_STREAM_URL, esp_http_client_get_status_code(client));
        r->client = client;
        r->pos = r->len = 0;

        int frames = 0;
        size_t bytes = 0;
        int64_t t0 = esp_timer_get_time();
        while (1) {
            /* Part headers: boundary line, Content-Type, Content-Length, empty line */
            int content_len = -1;
            int n;
            while ((n = read_line(r, line, sizeof(line))) >= 0) {
                if (strncasecmp(line, "Content-Length:", 15) == 0) {
                    content_len = atoi(line + 15);
                } else if (n == 0 && content_len >= 0) {
                    break;
                }
            }
            if (n < 0) {
                break; /* connection lost */
            }
            if (content_len <= 0 || content_len > (int)s_jpeg_in_size) {
                ESP_LOGW(TAG, "Frame size %d not supported", content_len);
                break;
            }
            if (read_exact(r, s_jpeg_in, content_len) < 0) {
                break;
            }
            show_jpeg(content_len);
            frames++;
            bytes += content_len;

            int64_t now = esp_timer_get_time();
            if (now - t0 >= 5000000) {
                float s = (now - t0) / 1e6f;
                ESP_LOGI(TAG, "%.1f fps displayed, %.0f kbit/s", frames / s, bytes * 8 / s / 1000);
                frames = 0;
                bytes = 0;
                t0 = now;
            }
        }
        ESP_LOGW(TAG, "Stream ended, reconnecting");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(display_init());

    jpeg_decode_engine_cfg_t eng_cfg = {.intr_priority = 0, .timeout_ms = 100};
    ESP_ERROR_CHECK(jpeg_new_decoder_engine(&eng_cfg, &s_jpeg));
    jpeg_decode_memory_alloc_cfg_t in_cfg = {.buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER};
    jpeg_decode_memory_alloc_cfg_t out_cfg = {.buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER};
    s_jpeg_in = jpeg_alloc_decoder_mem(JPEG_IN_SIZE, &in_cfg, &s_jpeg_in_size);
    s_jpeg_out = jpeg_alloc_decoder_mem(MAX_FRAME_W * MAX_FRAME_H * 2, &out_cfg, &s_jpeg_out_size);
    assert(s_jpeg_in && s_jpeg_out);

    wifi_init();
    xTaskCreatePinnedToCore(stream_task, "stream", 8192, NULL, 5, NULL, 1);
}
