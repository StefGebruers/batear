/*
 * gps_task.h — GY-NEO6MV2 (NEO-6M) GPS driver + RTC time sync + low-power sleep
 *
 * Runs on UART1 using PIN_GPS_RX (GPIO 2) and PIN_GPS_TX (GPIO 3).
 *
 * Features:
 *   1. Reads NMEA sentences ($GPRMC, $GPGGA) at 9600 baud.
 *   2. Synchronizes ESP32 system RTC clock to true GPS UTC time.
 *   3. Stores coordinates in NVS ("gps_cfg" namespace) so location is preserved
 *      across reboots even when indoors or in satellite shadow.
 *   4. Low-power standby: Once a 3D fix is obtained, puts NEO-6M into sleep
 *      mode (<0.5 mA) so it does not drain the battery.
 *   5. Wakes up periodically or on manual command ('gps fix').
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     has_fix;              /* True if valid GPS 2D/3D fix is currently held */
    double   latitude;             /* Decimal degrees (-90.0 .. +90.0) */
    double   longitude;            /* Decimal degrees (-180.0 .. +180.0) */
    float    altitude_m;           /* Altitude above mean sea level in meters */
    uint8_t  sats;                 /* Number of satellites used in fix */
    char     iso_time[32];         /* "YYYY-MM-DDTHH:MM:SSZ" format from GPS UTC */
    uint32_t last_fix_timestamp_s; /* System uptime (seconds) when last fix occurred */
    bool     is_sleeping;          /* True if NEO-6M is in low-power standby mode */
} GpsLocation_t;

/**
 * Initialize GPS UART (UART1 on PIN_GPS_RX=GPIO2, PIN_GPS_TX=GPIO3)
 * and launch the background GPS task.
 */
esp_err_t gps_task_init(void);

/**
 * Thread-safe copy of current GPS location and timestamp.
 * Returns true if valid coordinates are available (active fix or restored from NVS).
 */
bool gps_get_location(GpsLocation_t *out);

/**
 * Request GPS to wake up from low-power standby and acquire a fresh fix.
 */
void gps_request_fix(void);

#ifdef __cplusplus
}
#endif

