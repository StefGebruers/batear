/*
 * wifi_mqtt_task.cpp — Wi-Fi STA + MQTT + HA Discovery + OLED for Wi-Fi Detector
 *
 * Credentials are read from NVS namespace "wifi_det_cfg" first; if a key
 * is absent the compile-time Kconfig default is used instead.
 *
 * Runs on Core 0.  AudioTask (Core 1) sends DroneEvent_t items via
 * g_drone_event_queue; this task updates the OLED + LED and publishes JSON to:
 *   batear/nodes/<device_id>/status
 */

#include "wifi_mqtt_task.h"
#include "http_api.h"
#include "audio_processor.h"
#include "drone_detector.h"
#include "pin_config.h"
#include "lorawan_provision.h"
#include "oled.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "mqtt_client.h"

#include <cstring>
#include <cstdio>
#include <cmath>

static const char *TAG = "wifi_mqtt";

#define CFG_STR_MAX  128
#define DEVID_MAX     32

/* ---- runtime config (populated from NVS → Kconfig fallback) ---- */
static char s_wifi_ssid[CFG_STR_MAX];
static char s_wifi_pass[CFG_STR_MAX];
static char s_mqtt_url[CFG_STR_MAX];
static char s_mqtt_user[CFG_STR_MAX];
static char s_mqtt_pass[CFG_STR_MAX];
static char s_device_id[DEVID_MAX];
static char s_ip_str[24] = "0.0.0.0";

/* ---- WiFi event synchronisation ---- */
static EventGroupHandle_t s_wifi_eg;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

/* ---- MQTT state ---- */
static esp_mqtt_client_handle_t s_mqtt;
static bool s_mqtt_connected;

/* ---- pre-built topic strings ---- */
static char s_topic_avail[64];
static char s_topic_status[80];

/* ================================================================
 * RMS → dB helper
 * ================================================================ */

static uint8_t rms_to_db(float rms)
{
    if (rms <= 0.0f) return 0;
    float db = 20.0f * log10f(rms) + 120.0f;
    if (db < 0.0f) return 0;
    if (db > 255.0f) return 255;
    return (uint8_t)db;
}

/* ================================================================
 * OLED & LED helpers (for Heltec V3 / V4)
 * ================================================================ */

static void hw_display_init(void)
{
#if defined(PIN_LED)
    gpio_config_t led_cfg = {};
    led_cfg.pin_bit_mask = (1ULL << PIN_LED);
    led_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&led_cfg);
    gpio_set_level((gpio_num_t)PIN_LED, 0);
#endif

#if defined(PIN_OLED_SDA)
#if BOARD_HAS_VEXT
    gpio_config_t vext_cfg = {};
    vext_cfg.pin_bit_mask = (1ULL << PIN_VEXT);
    vext_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&vext_cfg);
    gpio_set_level((gpio_num_t)PIN_VEXT, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
#endif
    oled_init(PIN_OLED_SDA, PIN_OLED_SCL, PIN_OLED_RST);
    oled_clear();
    oled_print(0, 0, "BATEAR WIFI DET");
    oled_print(0, 2, "Booting...");
    oled_flush();
#endif
}

static void hw_display_status(const char *line2, const char *line4, const char *line6)
{
#if defined(PIN_OLED_SDA)
    oled_clear();
    oled_print(0, 0, "BATEAR WIFI DET");
    if (line2) oled_print(0, 2, line2);
    if (line4) oled_print(0, 4, line4);
    if (line6) oled_print(0, 6, line6);
    oled_flush();
#else
    (void)line2; (void)line4; (void)line6;
#endif
}

static void hw_display_event(const DroneEvent_t *ev)
{
    bool alarm = (ev->type == DRONE_EVENT_ALARM);
#if defined(PIN_LED)
    gpio_set_level((gpio_num_t)PIN_LED, alarm ? 1 : 0);
#endif

#if defined(PIN_OLED_SDA)
    char line[48];
    oled_clear();
    oled_print(0, 0, alarm ? "!! DRONE ALARM !!" : "BATEAR WIFI DET");

    snprintf(line, sizeof(line), "ID:%.8s %s", s_device_id,
             s_mqtt_connected ? "MQTT:OK" : "MQTT:--");
    oled_print(0, 2, line);

    unsigned f0_hz = (unsigned)((ev->f0_bin * AUDIO_PROC_SAMPLE_RATE_HZ) / AUDIO_PROC_FFT_SIZE);
    snprintf(line, sizeof(line), "dB:%u F0:%uHz",
             (unsigned)rms_to_db(ev->rms), f0_hz);
    oled_print(0, 4, line);

    snprintf(line, sizeof(line), "IP:%.15s", s_ip_str);
    oled_print(0, 6, line);
    oled_flush();
#endif
}

/* ================================================================
 * NVS config loader
 * ================================================================ */

static void load_nvs_str(nvs_handle_t h, const char *key,
                          char *dst, size_t dst_sz, const char *fallback)
{
    size_t len = dst_sz;
    if (nvs_get_str(h, key, dst, &len) != ESP_OK) {
        strncpy(dst, fallback, dst_sz - 1);
        dst[dst_sz - 1] = '\0';
    }
}

static void load_config(void)
{
    nvs_handle_t h;
    bool opened = (nvs_open("wifi_det_cfg", NVS_READONLY, &h) == ESP_OK);

    if (opened) {
        load_nvs_str(h, "wifi_ssid", s_wifi_ssid, sizeof(s_wifi_ssid),
                     CONFIG_BATEAR_WIFI_SSID);
        load_nvs_str(h, "wifi_pass", s_wifi_pass, sizeof(s_wifi_pass),
                     CONFIG_BATEAR_WIFI_PASS);
        load_nvs_str(h, "mqtt_url",  s_mqtt_url,  sizeof(s_mqtt_url),
                     CONFIG_BATEAR_MQTT_BROKER_URL);
        load_nvs_str(h, "mqtt_user", s_mqtt_user, sizeof(s_mqtt_user),
                     CONFIG_BATEAR_MQTT_USER);
        load_nvs_str(h, "mqtt_pass", s_mqtt_pass, sizeof(s_mqtt_pass),
                     CONFIG_BATEAR_MQTT_PASS);
        load_nvs_str(h, "device_id", s_device_id, DEVID_MAX,
                     CONFIG_BATEAR_WIFI_DEVICE_ID);
        nvs_close(h);
    } else {
        strncpy(s_wifi_ssid, CONFIG_BATEAR_WIFI_SSID, sizeof(s_wifi_ssid) - 1);
        strncpy(s_wifi_pass, CONFIG_BATEAR_WIFI_PASS, sizeof(s_wifi_pass) - 1);
        strncpy(s_mqtt_url,  CONFIG_BATEAR_MQTT_BROKER_URL, sizeof(s_mqtt_url) - 1);
        strncpy(s_mqtt_user, CONFIG_BATEAR_MQTT_USER, sizeof(s_mqtt_user) - 1);
        strncpy(s_mqtt_pass, CONFIG_BATEAR_MQTT_PASS, sizeof(s_mqtt_pass) - 1);
        strncpy(s_device_id, CONFIG_BATEAR_WIFI_DEVICE_ID, sizeof(s_device_id) - 1);
        ESP_LOGW(TAG, "NVS namespace 'wifi_det_cfg' not found — using Kconfig defaults");
    }

    snprintf(s_topic_avail, sizeof(s_topic_avail),
             "batear/nodes/%s/availability", s_device_id);
    snprintf(s_topic_status, sizeof(s_topic_status),
             "batear/nodes/%s/status", s_device_id);

    ESP_LOGI(TAG, "cfg: wifi_ssid='%s' mqtt_url='%s' device_id='%s'",
             s_wifi_ssid, s_mqtt_url, s_device_id);
}

/* ================================================================
 * WiFi STA
 * ================================================================ */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "WiFi disconnected — reconnecting...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        auto *ev = static_cast<ip_event_got_ip_t *>(data);
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&ev->ip_info.ip));
        ESP_LOGI(TAG, "WiFi Got IP: %s", s_ip_str);
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_init_sta(void)
{
    s_wifi_eg = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    esp_event_handler_instance_t h_wifi, h_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, &h_wifi));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, &h_ip));

    if (s_wifi_ssid[0] == '\0') {
        ESP_LOGW(TAG, "No WiFi SSID configured! Use serial console: set wifi_ssid <SSID> && set wifi_pass <PASS> && reboot");
        hw_display_status("No WiFi SSID!", "Use USB serial:", "set wifi_ssid");
        return false;
    }

    wifi_config_t sta_cfg = {};
    strncpy(reinterpret_cast<char *>(sta_cfg.sta.ssid),
            s_wifi_ssid, sizeof(sta_cfg.sta.ssid) - 1);
    strncpy(reinterpret_cast<char *>(sta_cfg.sta.password),
            s_wifi_pass, sizeof(sta_cfg.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    char line2[24];
    snprintf(line2, sizeof(line2), "WiFi: %.14s", s_wifi_ssid);
    hw_display_status(line2, "Connecting...", NULL);

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_eg, WIFI_CONNECTED_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(20000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected to %s (%s)", s_wifi_ssid, s_ip_str);
        return true;
    }
    ESP_LOGW(TAG, "WiFi initial connect timeout — continuing in background");
    return true;
}

/* ================================================================
 * HA MQTT Discovery
 * ================================================================ */

static void publish_ha_discovery(void)
{
    char topic[128];
    char payload[768];

    /* Binary sensor — drone detection */
    snprintf(topic, sizeof(topic),
             "homeassistant/binary_sensor/batear_%s/drone/config", s_device_id);

    snprintf(payload, sizeof(payload),
        "{"
            "\"name\":\"Batear %s Drone Detected\","
            "\"unique_id\":\"batear_%s_drone\","
            "\"device_class\":\"safety\","
            "\"state_topic\":\"%s\","
            "\"value_template\":\"{{ 'ON' if value_json.drone_detected else 'OFF' }}\","
            "\"availability_topic\":\"%s\","
            "\"payload_available\":\"online\","
            "\"payload_not_available\":\"offline\","
            "\"json_attributes_topic\":\"%s\","
            "\"device\":{"
                "\"identifiers\":[\"batear_%s\"],"
                "\"name\":\"Batear WiFi Detector %s\","
                "\"manufacturer\":\"Batear\","
                "\"model\":\"ESP32-S3 WiFi Detector\""
            "}"
        "}",
        s_device_id, s_device_id,
        s_topic_status, s_topic_avail, s_topic_status,
        s_device_id, s_device_id);

    esp_mqtt_client_publish(s_mqtt, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "HA discovery published → %s", topic);

    /* Confidence sensor */
    snprintf(topic, sizeof(topic),
             "homeassistant/sensor/batear_%s/confidence/config", s_device_id);

    snprintf(payload, sizeof(payload),
        "{"
            "\"name\":\"Batear %s Confidence\","
            "\"unique_id\":\"batear_%s_confidence\","
            "\"unit_of_measurement\":\"%%\","
            "\"state_topic\":\"%s\","
            "\"value_template\":\"{{ (value_json.confidence * 100) | round(1) }}\","
            "\"availability_topic\":\"%s\","
            "\"payload_available\":\"online\","
            "\"payload_not_available\":\"offline\","
            "\"entity_category\":\"diagnostic\","
            "\"device\":{\"identifiers\":[\"batear_%s\"]}"
        "}",
        s_device_id, s_device_id,
        s_topic_status, s_topic_avail,
        s_device_id);

    esp_mqtt_client_publish(s_mqtt, topic, payload, 0, 1, 1);

    /* Initial idle state so Home Assistant entities initialize immediately */
    char init_json[256];
    uint8_t det_id = lorawan_get_keys()->device_id;
    snprintf(init_json, sizeof(init_json),
        "{"
            "\"drone_detected\":false,"
            "\"detector_id\":%u,"
            "\"rms_db\":0,"
            "\"f0_bin\":0,"
            "\"seq\":0,"
            "\"confidence\":0.0000,"
            "\"timestamp\":0"
        "}",
        (unsigned)det_id);
    esp_mqtt_client_publish(s_mqtt, s_topic_status, init_json, 0, 1, 1);
}

/* ================================================================
 * MQTT event handler
 * ================================================================ */

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *data)
{
    (void)arg; (void)base; (void)event_id;
    auto *ev = static_cast<esp_mqtt_event_handle_t>(data);

    switch (ev->event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected to %s", s_mqtt_url);
        s_mqtt_connected = true;
        esp_mqtt_client_publish(s_mqtt, s_topic_avail, "online", 0, 1, 1);
        publish_ha_discovery();
        {
            char l2[48], l4[48];
            snprintf(l2, sizeof(l2), "ID:%.8s MQTT:OK", s_device_id);
            snprintf(l4, sizeof(l4), "IP:%.15s", s_ip_str);
            hw_display_status(l2, l4, "Listening (16kHz)");
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        s_mqtt_connected = false;
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error type=%d", ev->error_handle->error_type);
        break;

    default:
        break;
    }
}

static void mqtt_start(void)
{
    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri  = s_mqtt_url;
    cfg.credentials.username = s_mqtt_user;
    cfg.credentials.authentication.password = s_mqtt_pass;

    cfg.session.last_will.topic   = s_topic_avail;
    cfg.session.last_will.msg     = "offline";
    cfg.session.last_will.msg_len = 7;
    cfg.session.last_will.qos     = 1;
    cfg.session.last_will.retain  = 1;

    s_mqtt = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_mqtt, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt);
}

/* ================================================================
 * Task entry
 * ================================================================ */

extern "C" void WifiMqttTask(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "WifiMqttTask start (core %d)", xPortGetCoreID());

    hw_display_init();
    load_config();

    if (wifi_init_sta()) {
        mqtt_start();
        http_api_start();
    }

    DroneEvent_t ev;
    char json[320];
    uint16_t tx_seq = 1;

    for (;;) {
        if (xQueueReceive(g_drone_event_queue, &ev, pdMS_TO_TICKS(5000)) == pdTRUE) {
            hw_display_event(&ev);

            int64_t ts = esp_timer_get_time() / 1000000LL;
            uint8_t det_id = lorawan_get_keys()->device_id;

            HttpDroneStatus_t hst = {
                .drone_detected = (ev.type == DRONE_EVENT_ALARM),
                .detector_id    = det_id,
                .rms_db         = rms_to_db(ev.rms),
                .f0_bin         = static_cast<uint16_t>(ev.f0_bin),
                .confidence     = ev.peak_ratio,
                .timestamp      = ts,
            };
            http_api_update_status(&hst);

            if (!s_mqtt_connected) {
                ESP_LOGW(TAG, "MQTT not connected — event shown on OLED only");
                continue;
            }

            snprintf(json, sizeof(json),
                "{"
                    "\"drone_detected\":%s,"
                    "\"detector_id\":%u,"
                    "\"rms_db\":%u,"
                    "\"f0_bin\":%u,"
                    "\"seq\":%u,"
                    "\"confidence\":%.4f,"
                    "\"timestamp\":%lld"
                "}",
                (ev.type == DRONE_EVENT_ALARM) ? "true" : "false",
                (unsigned)det_id,
                (unsigned)rms_to_db(ev.rms), (unsigned)ev.f0_bin,
                (unsigned)tx_seq++,
                (double)ev.peak_ratio,
                (long long)ts);

            esp_mqtt_client_publish(s_mqtt, s_topic_status, json, 0, 1, 1);
            ESP_LOGI(TAG, "pub → %s : %s", s_topic_status, json);
        }
    }
}

