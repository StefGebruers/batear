/*
 * gps_task.c — GY-NEO6MV2 (NEO-6M) GPS driver + RTC time sync + low-power sleep
 */

#include "gps_task.h"
#include "pin_config.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

static const char *TAG = "gps";

#define GPS_UART_NUM        UART_NUM_1
#define GPS_UART_BAUD_RATE  9600
#define GPS_BUF_SIZE        512

static SemaphoreHandle_t s_gps_mutex   = NULL;
static TaskHandle_t      s_gps_task_h  = NULL;
static GpsLocation_t     s_loc         = {0};
static bool              s_has_stored  = false;

/* Convert Gregorian date/time to UTC Unix timestamp without timezone dependency */
static time_t date_to_epoch(int year, int mon, int day, int hour, int min, int sec)
{
    if (mon < 3) {
        year--;
        mon += 12;
    }
    int64_t days = 365LL * year + year / 4 - year / 100 + year / 400 + (153 * (mon - 3) + 2) / 5 + day - 719469;
    return (time_t)(days * 86400LL + hour * 3600 + min * 60 + sec);
}

static double nmea_to_decimal(const char *deg_min_str, char dir)
{
    if (!deg_min_str || !*deg_min_str) {
        return 0.0;
    }
    double val = atof(deg_min_str);
    int deg = (int)(val / 100.0);
    double min = val - (deg * 100.0);
    double res = (double)deg + (min / 60.0);
    if (dir == 'S' || dir == 'W') {
        res = -res;
    }
    return res;
}

static bool verify_nmea_checksum(const char *line)
{
    if (line == NULL || line[0] != '$') {
        return false;
    }
    const char *asterisk = strchr(line, '*');
    if (!asterisk) {
        return false;
    }
    uint8_t calc_cs = 0;
    for (const char *p = line + 1; p < asterisk; p++) {
        calc_cs ^= (uint8_t)(*p);
    }
    char *endptr = NULL;
    long given_cs = strtol(asterisk + 1, &endptr, 16);
    return (endptr != asterisk + 1) && (calc_cs == (uint8_t)given_cs);
}

/* Save latest valid coordinates to NVS so location survives reboots without satellites */
static void save_location_to_nvs(double lat, double lon, float alt)
{
    nvs_handle_t h;
    if (nvs_open("gps_cfg", NVS_READWRITE, &h) == ESP_OK) {
        int32_t lat_e6 = (int32_t)(lat * 1000000.0);
        int32_t lon_e6 = (int32_t)(lon * 1000000.0);
        int16_t alt_m  = (int16_t)alt;
        nvs_set_i32(h, "lat_e6", lat_e6);
        nvs_set_i32(h, "lon_e6", lon_e6);
        nvs_set_i16(h, "alt_m",  alt_m);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Saved location to NVS: lat=%.6f lon=%.6f alt=%.1fm", lat, lon, (double)alt);
    }
}

static void load_location_from_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open("gps_cfg", NVS_READONLY, &h) == ESP_OK) {
        int32_t lat_e6 = 0;
        int32_t lon_e6 = 0;
        int16_t alt_m  = 0;
        if (nvs_get_i32(h, "lat_e6", &lat_e6) == ESP_OK &&
            nvs_get_i32(h, "lon_e6", &lon_e6) == ESP_OK) {
            nvs_get_i16(h, "alt_m", &alt_m);
            if (s_gps_mutex && xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                s_loc.latitude    = (double)lat_e6 / 1000000.0;
                s_loc.longitude   = (double)lon_e6 / 1000000.0;
                s_loc.altitude_m  = (float)alt_m;
                s_loc.has_fix     = false; /* Stored from NVS, not live active fix */
                s_has_stored      = true;
                xSemaphoreGive(s_gps_mutex);
            }
            ESP_LOGI(TAG, "Restored last-known GPS coordinates from NVS: lat=%.6f lon=%.6f alt=%.1fm",
                     s_loc.latitude, s_loc.longitude, (double)s_loc.altitude_m);
        }
        nvs_close(h);
    }
}

static void put_neo6m_to_sleep(void)
{
    /* 1. UBX-RXM-PMREQ: Put u-blox receiver into backup mode (infinite duration) */
    const uint8_t ubx_sleep[] = {
        0xB5, 0x62,                         /* Header */
        0x02, 0x41,                         /* Class: RXM, ID: PMREQ */
        0x08, 0x00,                         /* Payload length = 8 */
        0x00, 0x00, 0x00, 0x00,             /* Duration = 0 (infinite) */
        0x02, 0x00, 0x00, 0x00,             /* Flags: 0x02 = backup mode */
        0x4D, 0x3B                          /* Checksum */
    };
    uart_write_bytes(GPS_UART_NUM, (const char *)ubx_sleep, sizeof(ubx_sleep));

    /* 2. PUBX standby command (fallback for NMEA mode) */
    const char *pubx_sleep = "$PUBX,41,1,0007,0003,9600,0*10\r\n";
    uart_write_bytes(GPS_UART_NUM, pubx_sleep, strlen(pubx_sleep));

    if (s_gps_mutex && xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_loc.is_sleeping = true;
        xSemaphoreGive(s_gps_mutex);
    }
    ESP_LOGI(TAG, "NEO-6M put into low-power sleep mode (<0.5 mA)");
}

static void wake_neo6m(void)
{
    /* Sending characters on UART TX toggles GPS RX pin and wakes up the NEO-6M */
    const char wake_chars[] = "\r\n\r\n\xFF\xFF\r\n";
    uart_write_bytes(GPS_UART_NUM, wake_chars, sizeof(wake_chars) - 1);
    if (s_gps_mutex && xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_loc.is_sleeping = false;
        xSemaphoreGive(s_gps_mutex);
    }
    ESP_LOGI(TAG, "Sent wake-up sequence to NEO-6M");
}

/* Parse NMEA lines: returns true if a valid fix update occurred */
static bool parse_nmea_sentence(char *line, int *consecutive_fixes)
{
    if (!verify_nmea_checksum(line)) {
        return false;
    }

    char *asterisk = strchr(line, '*');
    if (asterisk) {
        *asterisk = '\0';
    }

    /* Split CSV tokens */
    char *tokens[20] = {0};
    int tok_idx = 0;
    char *p = line;
    while (p && tok_idx < 20) {
        tokens[tok_idx++] = p;
        char *comma = strchr(p, ',');
        if (!comma) break;
        *comma = '\0';
        p = comma + 1;
    }

    if (tok_idx < 2) {
        return false;
    }

    /* Process $GPRMC or $GNRMC */
    if (strcmp(tokens[0], "$GPRMC") == 0 || strcmp(tokens[0], "$GNRMC") == 0) {
        if (tok_idx >= 10 && tokens[2][0] == 'A') { /* Status 'A' = Valid fix */
            const char *time_str = tokens[1];
            const char *lat_str  = tokens[3];
            const char lat_dir   = tokens[4][0];
            const char *lon_str  = tokens[5];
            const char lon_dir   = tokens[6][0];
            const char *date_str = tokens[9];

            double lat = nmea_to_decimal(lat_str, lat_dir);
            double lon = nmea_to_decimal(lon_str, lon_dir);

            /* Parse date (ddmmyy) and time (hhmmss.ss) */
            int hour = 0, min = 0, sec = 0;
            if (strlen(time_str) >= 6) {
                char tmp[3] = {0};
                tmp[0] = time_str[0]; tmp[1] = time_str[1]; hour = atoi(tmp);
                tmp[0] = time_str[2]; tmp[1] = time_str[3]; min  = atoi(tmp);
                tmp[0] = time_str[4]; tmp[1] = time_str[5]; sec  = atoi(tmp);
            }

            int day = 0, mon = 0, year = 0;
            if (strlen(date_str) >= 6) {
                char tmp[3] = {0};
                tmp[0] = date_str[0]; tmp[1] = date_str[1]; day  = atoi(tmp);
                tmp[0] = date_str[2]; tmp[1] = date_str[3]; mon  = atoi(tmp);
                tmp[0] = date_str[4]; tmp[1] = date_str[5]; year = atoi(tmp) + 2000;
            }

            /* Synchronize system RTC clock to GPS UTC time */
            if (year >= 2024 && mon >= 1 && mon <= 12 && day >= 1 && day <= 31) {
                time_t epoch = date_to_epoch(year, mon, day, hour, min, sec);
                struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
                settimeofday(&tv, NULL);

                if (s_gps_mutex && xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    snprintf(s_loc.iso_time, sizeof(s_loc.iso_time),
                             "%04d-%02d-%02dT%02d:%02d:%02dZ",
                             year, mon, day, hour, min, sec);
                    s_loc.has_fix              = true;
                    s_loc.latitude             = lat;
                    s_loc.longitude            = lon;
                    s_loc.last_fix_timestamp_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
                    xSemaphoreGive(s_gps_mutex);
                }
                (*consecutive_fixes)++;
                return true;
            }
        } else {
            /* Void fix */
            if (s_gps_mutex && xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                s_loc.has_fix = false;
                xSemaphoreGive(s_gps_mutex);
            }
        }
    }

    /* Process $GPGGA or $GNGGA */
    if (strcmp(tokens[0], "$GPGGA") == 0 || strcmp(tokens[0], "$GNGGA") == 0) {
        if (tok_idx >= 10) {
            int quality = atoi(tokens[6]);
            int sats    = atoi(tokens[7]);
            float alt   = (float)atof(tokens[9]);

            if (quality > 0) {
                if (s_gps_mutex && xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    s_loc.has_fix    = true;
                    s_loc.sats       = (uint8_t)sats;
                    s_loc.altitude_m = alt;
                    xSemaphoreGive(s_gps_mutex);
                }
                return true;
            }
        }
    }

    return false;
}

static void gps_worker_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "GPS task running (UART1 on RX=GPIO%d TX=GPIO%d baud=%d)",
             PIN_GPS_RX, PIN_GPS_TX, GPS_UART_BAUD_RATE);

    load_location_from_nvs();

    uint8_t rx_byte;
    char line_buf[128];
    int line_pos = 0;
    int consecutive_fixes = 0;
    const int FIXES_BEFORE_SLEEP = 8; /* Wait for 8 consecutive valid fixes (~8 sec) for accuracy */

    for (;;) {
        /* Check if we were requested to wake up */
        ulTaskNotifyTake(pdTRUE, 0);

        int len = uart_read_bytes(GPS_UART_NUM, &rx_byte, 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            if (rx_byte == '$') {
                line_pos = 0;
                line_buf[line_pos++] = '$';
            } else if (line_pos > 0) {
                if (rx_byte == '\r' || rx_byte == '\n') {
                    line_buf[line_pos] = '\0';
                    if (parse_nmea_sentence(line_buf, &consecutive_fixes)) {
                        if (consecutive_fixes >= FIXES_BEFORE_SLEEP) {
                            GpsLocation_t cur;
                            gps_get_location(&cur);
                            ESP_LOGI(TAG, "3D Fix locked: lat=%.6f lon=%.6f alt=%.1fm sats=%u time=%s",
                                     cur.latitude, cur.longitude, (double)cur.altitude_m,
                                     (unsigned)cur.sats, cur.iso_time);
                            save_location_to_nvs(cur.latitude, cur.longitude, cur.altitude_m);

                            /* Put NEO-6M into sleep mode to preserve battery */
                            put_neo6m_to_sleep();
                            consecutive_fixes = 0;

                            /* Sleep for 24 hours unless woken up manually via gps_request_fix() */
                            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(24ULL * 3600ULL * 1000ULL));
                            wake_neo6m();
                        }
                    }
                    line_pos = 0;
                } else if (line_pos < (int)sizeof(line_buf) - 1) {
                    line_buf[line_pos++] = (char)rx_byte;
                } else {
                    line_pos = 0; /* Buffer overflow guard */
                }
            }
        }
    }
}

esp_err_t gps_task_init(void)
{
    s_gps_mutex = xSemaphoreCreateMutex();
    if (s_gps_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    uart_config_t uart_cfg = {
        .baud_rate  = GPS_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(GPS_UART_NUM, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    /* ESP32 TX pin (GPIO 3) -> GPS RX; ESP32 RX pin (GPIO 2) -> GPS TX */
    err = uart_set_pin(GPS_UART_NUM, PIN_GPS_TX, PIN_GPS_RX,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_driver_install(GPS_UART_NUM, GPS_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(
        gps_worker_task, "GpsTask",
        4 * 1024 / sizeof(StackType_t),
        NULL, configMAX_PRIORITIES - 5,
        &s_gps_task_h, 0);

    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}

bool gps_get_location(GpsLocation_t *out)
{
    if (out == NULL) return false;
    bool has_data = false;
    if (s_gps_mutex && xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        *out = s_loc;
        has_data = (s_loc.has_fix || s_has_stored);
        xSemaphoreGive(s_gps_mutex);
    }
    return has_data;
}

void gps_request_fix(void)
{
    wake_neo6m();
    if (s_gps_task_h != NULL) {
        xTaskNotifyGive(s_gps_task_h);
    }
}
