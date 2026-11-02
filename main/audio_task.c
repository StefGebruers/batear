/*
 * audio_task.c — I2S + ESP-DSP FFT harmonic drone detector (Core 1)
 */

#include "audio_task.h"
#include "audio_processor.h"
#include "doa_processor.h"
#include "drone_detector.h"
#include "pin_config.h"

#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "nvs.h"
#include "sdkconfig.h"

#if CONFIG_BATEAR_ROLE_WIRED_DETECTOR
#include "tf_recorder.h"
#endif

static const char *TAG = "audio";

#define I2S_MIC_BCLK_GPIO   ((gpio_num_t)PIN_I2S_BCLK)
#define I2S_MIC_WS_GPIO     ((gpio_num_t)PIN_I2S_WS)
#define I2S_MIC_DIN_GPIO    ((gpio_num_t)PIN_I2S_DIN)
#define I2S_MIC_DIN2_GPIO   ((gpio_num_t)PIN_I2S_DIN2)

#define SAMPLE_RATE_HZ      AUDIO_PROC_SAMPLE_RATE_HZ
#define FRAME_SAMPLES       AUDIO_PROC_FFT_SIZE
#define HOP_MS              10

/* Fundamental search band (Hz); 3×f0 must stay below Nyquist (8 kHz). */
#define HARM_F0_MIN_HZ      110.f
#define HARM_F0_MAX_HZ      2400.f

#define EMA_ALPHA           0.30f
#define CONF_ON             0.30f
#define CONF_OFF            0.18f
#define SUSTAIN_FRAMES_ON   3
#define SUSTAIN_FRAMES_OFF  8
#define RMS_MIN             0.00025f

static float __attribute__((aligned(16))) s_fft_work[2 * FRAME_SAMPLES];
static int32_t __attribute__((aligned(16))) s_raw[FRAME_SAMPLES];
static int32_t __attribute__((aligned(16))) s_raw_right[FRAME_SAMPLES];
static int32_t __attribute__((aligned(16))) s_raw_ch2[FRAME_SAMPLES];
static int32_t __attribute__((aligned(16))) s_raw_ch3[FRAME_SAMPLES];
/* Stereo DMA: L,R pairs — ESP32-S3 Philips RX (see i2s_microphone_init). */
static int32_t __attribute__((aligned(16))) s_stereo[FRAME_SAMPLES * 2];
static int32_t __attribute__((aligned(16))) s_stereo1[FRAME_SAMPLES * 2];

#if CONFIG_BATEAR_ROLE_WIRED_DETECTOR
/* Down-sampled buffer handed off to the TF recorder — int16 mono @ 16 kHz.
 * The 32-bit I2S word from the ICS-43434 carries 24 valid bits in the upper
 * portion; right-shift by 16 keeps the most significant 16 bits. */
static int16_t __attribute__((aligned(16))) s_pcm16[FRAME_SAMPLES];
#endif

static i2s_chan_handle_t s_rx  = NULL;
static i2s_chan_handle_t s_rx1 = NULL;
static float             s_conf_ema = 0.f;

static esp_err_t i2s_microphone_init(void)
{
    /*
     * 4-Mic Cross Array ("kruisverband") on ESP32-S3:
     *   - I2S_NUM_0 (MASTER) drives BCLK (GPIO 4) and WS (GPIO 5) for all 4 mics,
     *     and reads Mic 0 (Noord, SEL=GND) + Mic 1 (Zuid, SEL=3V3) on DIN (GPIO 6).
     *   - I2S_NUM_1 (SLAVE) shares the exact same BCLK (GPIO 4) and WS (GPIO 5)
     *     clocks via internal GPIO matrix input, and reads Mic 2 (Oost, SEL=GND) +
     *     Mic 3 (West, SEL=3V3) on DIN2 (GPIO 7).
     */
    i2s_chan_config_t chan0_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan0_cfg.dma_desc_num  = 8;
    chan0_cfg.dma_frame_num = 256;

    esp_err_t err = i2s_new_channel(&chan0_cfg, NULL, &s_rx);
    if (err != ESP_OK) {
        return err;
    }

    i2s_std_config_t std0_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                         I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk  = I2S_GPIO_UNUSED,
            .bclk  = I2S_MIC_BCLK_GPIO,
            .ws    = I2S_MIC_WS_GPIO,
            .dout  = I2S_GPIO_UNUSED,
            .din   = I2S_MIC_DIN_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    std0_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384;

    err = i2s_channel_init_std_mode(s_rx, &std0_cfg);
    if (err != ESP_OK) {
        i2s_del_channel(s_rx);
        s_rx = NULL;
        return err;
    }

    /* INMP441 / ICS-43434 datasheets require a pull-down on SD (DOUT) during
     * High-Z tri-state slots; disable ESP32-S3's default internal pull-up.
     * Also reduce BCLK/WS pad drive strength to GPIO_DRIVE_CAP_0 (~5 mA) to
     * source-terminate daisy-chained SCK/WS jumper wires and prevent ringing. */
    gpio_pullup_dis(I2S_MIC_DIN_GPIO);
    gpio_pulldown_en(I2S_MIC_DIN_GPIO);
    gpio_set_drive_capability(I2S_MIC_BCLK_GPIO, GPIO_DRIVE_CAP_0);
    gpio_set_drive_capability(I2S_MIC_WS_GPIO, GPIO_DRIVE_CAP_0);

    /* Initialize second stereo bus (I2S_NUM_1) as Slave clocked by GPIO 4 & GPIO 5 */
    i2s_chan_config_t chan1_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
    chan1_cfg.dma_desc_num  = 8;
    chan1_cfg.dma_frame_num = 256;

    if (i2s_new_channel(&chan1_cfg, NULL, &s_rx1) == ESP_OK) {
        i2s_std_config_t std1_cfg = std0_cfg;
        std1_cfg.gpio_cfg.din = I2S_MIC_DIN2_GPIO;
        if (i2s_channel_init_std_mode(s_rx1, &std1_cfg) == ESP_OK) {
            gpio_pullup_dis(I2S_MIC_DIN2_GPIO);
            gpio_pulldown_en(I2S_MIC_DIN2_GPIO);
            /* Enable slave first so it starts on the very first clock edge from master */
            i2s_channel_enable(s_rx1);
        } else {
            i2s_del_channel(s_rx1);
            s_rx1 = NULL;
        }
    }

    return i2s_channel_enable(s_rx);
}

/**
 * Block until one full FRAME_SAMPLES mono int32 frame is available.
 * Accumulates partial reads so FreeRTOS I2S DMA chunking never drops samples.
 */
/** First-frame + rare heartbeat so bring-up shows mic activity (not only on ALARM). */
static float raw_i32_rms_norm(const int32_t *s, int n)
{
    if (n <= 0 || s == NULL) {
        return 0.f;
    }
    double acc = 0.0;
    for (int i = 0; i < n; i++) {
        const double v = (double)s[i] / 2147483648.0;
        acc += v * v;
    }
    return (float)sqrt(acc / (double)n);
}

static void log_pcm_int32_span(const int32_t *s, int n, float raw_rms_norm, const char *note)
{
    if (n <= 0 || s == NULL) {
        return;
    }
    int32_t lo = s[0], hi = s[0];
    for (int i = 1; i < n; i++) {
        if (s[i] < lo) {
            lo = s[i];
        }
        if (s[i] > hi) {
            hi = s[i];
        }
    }
    ESP_LOGI(TAG, "mic %s: int32 min=%" PRId32 " max=%" PRId32 " span=%" PRId32 " raw_rms~%.5f",
             note, lo, hi, (int32_t)(hi - lo), raw_rms_norm);
}

static bool read_pcm_chunk_bytes(i2s_chan_handle_t chan, uint8_t *dst, size_t need_bytes, uint32_t timeout_ms)
{
    if (chan == NULL) {
        return false;
    }
    size_t got = 0;
    while (got < need_bytes) {
        size_t    chunk = 0;
        esp_err_t e     = i2s_channel_read(chan, dst + got, need_bytes - got, &chunk, pdMS_TO_TICKS(timeout_ms));
        if (e != ESP_OK) {
            return false;
        }
        if (chunk == 0) {
            vTaskDelay(1);
            continue;
        }
        got += chunk;
    }
    return true;
}

/**
 * Sanitize raw 32-bit I2S slot word from 24-bit MEMS microphones (INMP441 / ICS-43434):
 *   1. Sign-extend from bit 28 into bits [31:29]:
 *      When two INMP441s share one SD line, the first MSB clock edge after the
 *      8-bit High-Z tri-state gap can retain the bus pull state (1 or 0), causing
 *      full-scale 0x80000000 spikes (RMS ~0.578 or ~0.044) on zero-crossings.
 *      Bits [28:8] are 100% settled and preserve full dynamic range up to 102 dB SPL.
 *   2. Mask off bits [7:0], which are High-Z tri-state padding after the 24-bit word.
 */
static inline int32_t sanitize_i2s_24bit_sample(int32_t raw)
{
    int32_t s = ((int32_t)((uint32_t)raw << 7)) >> 5;
    return s & ~0xFF;
}

/** Read 4 channels from I2S_NUM_0 (DIN=GPIO6: Ch0, Ch1) and I2S_NUM_1 (DIN2=GPIO7: Ch2, Ch3). */
static bool read_pcm_frame_quad_i32(int32_t *out_ch0, int32_t *out_ch1,
                                    int32_t *out_ch2, int32_t *out_ch3, int n)
{
    if (n != FRAME_SAMPLES) {
        return false;
    }
    const size_t need_bytes = (size_t)n * 2u * sizeof(int32_t);
    if (!read_pcm_chunk_bytes(s_rx, (uint8_t *)s_stereo, need_bytes, 1000)) {
        ESP_LOGW(TAG, "I2S0 read timeout");
        return false;
    }
#if CONFIG_BATEAR_I2S_MIC_SLOT_RIGHT
    const unsigned ch_l = 1u;
    const unsigned ch_r = 0u;
#else
    const unsigned ch_l = 0u;
    const unsigned ch_r = 1u;
#endif
    for (int i = 0; i < n; i++) {
        out_ch0[i] = sanitize_i2s_24bit_sample(s_stereo[i * 2 + ch_l]);
        out_ch1[i] = sanitize_i2s_24bit_sample(s_stereo[i * 2 + ch_r]);
    }

    if (s_rx1 != NULL && read_pcm_chunk_bytes(s_rx1, (uint8_t *)s_stereo1, need_bytes, 100)) {
        for (int i = 0; i < n; i++) {
            out_ch2[i] = sanitize_i2s_24bit_sample(s_stereo1[i * 2 + ch_l]);
            out_ch3[i] = sanitize_i2s_24bit_sample(s_stereo1[i * 2 + ch_r]);
        }
    } else {
        memset(out_ch2, 0, (size_t)n * sizeof(int32_t));
        memset(out_ch3, 0, (size_t)n * sizeof(int32_t));
    }
    return true;
}

static void push_event(DroneEventType type, float ratio, int active, float rms,
                       int16_t doa_deg, int16_t doa_elev_deg, uint8_t active_mics, float doa_conf)
{
    DroneEvent_t ev = {
        .type         = type,
        .peak_ratio   = ratio,
        .f0_bin       = active,
        .rms          = rms,
        .doa_deg      = doa_deg,
        .doa_elev_deg = doa_elev_deg,
        .active_mics  = active_mics,
        .doa_conf     = doa_conf,
        .timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
    };
    if (xQueueSend(g_drone_event_queue, &ev, 0) != pdTRUE) {
        ESP_LOGW(TAG, "event queue full — dropped 0x%02X", (unsigned)type);
    }
}

void AudioTask(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "AudioTask start (core %d) FFT N=%d", xPortGetCoreID(), FRAME_SAMPLES);

    {
        nvs_handle_t h;
        if (nvs_open("lora_cfg", NVS_READONLY, &h) == ESP_OK) {
            uint8_t sens = 3;
            if (nvs_get_u8(h, "sensitivity", &sens) == ESP_OK) {
                audio_processor_set_sensitivity((int)sens);
            }
            uint8_t gain = 2;
            if (nvs_get_u8(h, "mic_gain", &gain) == ESP_OK) {
                audio_processor_set_gain((float)gain);
            }
            uint16_t spacing = 83;
            if (nvs_get_u16(h, "mic_spacing", &spacing) == ESP_OK) {
                doa_processor_set_spacing_mm((float)spacing);
            }
            nvs_close(h);
        }
    }

    esp_err_t err = audio_processor_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "audio_processor_init failed — suspending");
        vTaskSuspend(NULL);
        return;
    }

    err = doa_processor_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "doa_processor_init failed: %s — continuing without DoA", esp_err_to_name(err));
    }

    err = i2s_microphone_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S init failed: %s — suspending", esp_err_to_name(err));
        doa_processor_deinit();
        audio_processor_deinit();
        vTaskSuspend(NULL);
        return;
    }
    ESP_LOGI(TAG, "I2S OK — 4-mic Cross DoA BCLK=%d WS=%d DIN0=%d DIN1=%d spacing=%.0fmm",
             PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DIN, PIN_I2S_DIN2, (double)doa_processor_get_spacing_mm());

    int     sustain_on      = 0;
    int     sustain_off     = 0;
    bool    alarm           = false;
    int     detection_count = 0;
    bool    mic_diag_logged = false;
    int     frames_total    = 0;
    int16_t last_pushed_doa  = -1;
    int16_t last_pushed_elev = -1;
    int64_t last_push_us     = 0;

    for (;;) {
        if (!read_pcm_frame_quad_i32(s_raw, s_raw_right, s_raw_ch2, s_raw_ch3, FRAME_SAMPLES)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (!mic_diag_logged) {
            log_pcm_int32_span(s_raw,       FRAME_SAMPLES, raw_i32_rms_norm(s_raw,       FRAME_SAMPLES), "Mic0 Noord (DIN=6, SEL=GND)");
            log_pcm_int32_span(s_raw_right, FRAME_SAMPLES, raw_i32_rms_norm(s_raw_right, FRAME_SAMPLES), "Mic1 Zuid  (DIN=6, SEL=3V3)");
            log_pcm_int32_span(s_raw_ch2,   FRAME_SAMPLES, raw_i32_rms_norm(s_raw_ch2,   FRAME_SAMPLES), "Mic2 Oost  (DIN=7, SEL=GND)");
            log_pcm_int32_span(s_raw_ch3,   FRAME_SAMPLES, raw_i32_rms_norm(s_raw_ch3,   FRAME_SAMPLES), "Mic3 West  (DIN=7, SEL=3V3)");
            mic_diag_logged = true;
        }

#if CONFIG_BATEAR_ROLE_WIRED_DETECTOR
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            s_pcm16[i] = (int16_t)(s_raw[i] >> 16);
        }
        tf_recorder_push_pcm(s_pcm16, FRAME_SAMPLES);
#endif

#if CONFIG_BATEAR_AUDIO_PERF_LOG
        const int64_t t_psd_start = esp_timer_get_time();
#endif
        err = audio_processor_compute_psd(s_fft_work, s_raw, FRAME_SAMPLES);
#if CONFIG_BATEAR_AUDIO_PERF_LOG
        const int32_t dt_psd_us = (int32_t)(esp_timer_get_time() - t_psd_start);
#endif
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "compute_psd: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(HOP_MS));
            continue;
        }

        const float *psd = audio_processor_last_psd();
        const float rms  = audio_processor_last_rms();

        frames_total++;
        if ((frames_total % 300) == 0) {
            ESP_LOGI(TAG, "mic heartbeat (~30s): win_rms=%.5f (gate %.4f)", rms, RMS_MIN);
            /* Silence-recovery policy:
             *   - 1st silent heartbeat (~30 s): restart the I2S channel once
             *     in case the ESP-side DMA is stuck.
             *   - Subsequent silent heartbeats: just warn. The ICS-43434 can
             *     latch up after a 3.3 V transient (e.g. SD card yank); the
             *     mic recovers on its own in 1–3 minutes when its internal
             *     brown-out logic releases. ESP-side I2S restarts don't help
             *     because we can't power-cycle the mic in software. */
            static int  silent_streak = 0;
            static bool restart_tried = false;
            if (rms < 1e-6f) {
                silent_streak++;
                if (silent_streak == 1 && !restart_tried) {
                    ESP_LOGW(TAG, "I2S silent — trying one channel restart");
                    esp_err_t e1 = i2s_channel_disable(s_rx);
                    esp_err_t e2 = i2s_channel_enable(s_rx);
                    ESP_LOGI(TAG, "I2S restart: disable=%s enable=%s",
                             esp_err_to_name(e1), esp_err_to_name(e2));
                    mic_diag_logged = false;  /* re-log first frame after restart */
                    restart_tried   = true;
                } else {
                    ESP_LOGW(TAG,
                             "I2S still silent — likely ICS-43434 latch-up after a power "
                             "transient (SD yank). Recovers on its own in 1–3 min, or RST the board.");
                }
            } else {
                silent_streak = 0;
                restart_tried = false;  /* re-arm for the next event */
            }
        }

        HarmonicAnalysisResult hr;
#if CONFIG_BATEAR_AUDIO_PERF_LOG
        const int64_t t_harm_start = esp_timer_get_time();
#endif
        bool harm_ok =
            analyze_harmonics(psd, AUDIO_PROC_PSD_BINS, HARM_F0_MIN_HZ, HARM_F0_MAX_HZ, &hr);
        if (!harm_ok) {
            /* Also check orthogonal cross arm (Mic 2 Oost) if Mic 0 Noord missed */
            HarmonicAnalysisResult hr_orth;
            if (audio_processor_compute_psd(s_fft_work, s_raw_ch2, FRAME_SAMPLES) == ESP_OK) {
                const float *psd_orth = audio_processor_last_psd();
                bool ok_orth = analyze_harmonics(psd_orth, AUDIO_PROC_PSD_BINS,
                                                 HARM_F0_MIN_HZ, HARM_F0_MAX_HZ, &hr_orth);
                if (ok_orth || hr_orth.confidence > hr.confidence) {
                    hr      = hr_orth;
                    harm_ok = ok_orth;
                }
            }
        }
#if CONFIG_BATEAR_AUDIO_PERF_LOG
        const int32_t dt_harm_us = (int32_t)(esp_timer_get_time() - t_harm_start);
        static int      s_perf_n;
        static int32_t  s_psd_sum, s_harm_sum, s_psd_min = INT32_MAX, s_psd_max, s_harm_min = INT32_MAX,
            s_harm_max;
        s_psd_sum += dt_psd_us;
        s_harm_sum += dt_harm_us;
        if (dt_psd_us < s_psd_min) {
            s_psd_min = dt_psd_us;
        }
        if (dt_psd_us > s_psd_max) {
            s_psd_max = dt_psd_us;
        }
        if (dt_harm_us < s_harm_min) {
            s_harm_min = dt_harm_us;
        }
        if (dt_harm_us > s_harm_max) {
            s_harm_max = dt_harm_us;
        }
        s_perf_n++;
        if (s_perf_n >= 100) {
            ESP_LOGI(TAG,
                     "perf (100 frames): psd_fft min/avg/max=%" PRId32 "/%" PRId32 "/%" PRId32 " us  "
                     "harm min/avg/max=%" PRId32 "/%" PRId32 "/%" PRId32 " us  dsp_tot_avg=%" PRId32 " us",
                     s_psd_min, s_psd_sum / s_perf_n, s_psd_max, s_harm_min, s_harm_sum / s_perf_n, s_harm_max,
                     (s_psd_sum + s_harm_sum) / s_perf_n);
            s_perf_n     = 0;
            s_psd_sum    = 0;
            s_harm_sum   = 0;
            s_psd_min    = INT32_MAX;
            s_psd_max    = 0;
            s_harm_min   = INT32_MAX;
            s_harm_max   = 0;
        }
#endif

        float conf_display  = s_conf_ema;
        const int64_t now_us = esp_timer_get_time();

        DoaResult doa_res = {0};
        const int32_t *pcm_ch[4] = { s_raw, s_raw_right, s_raw_ch2, s_raw_ch3 };
        doa_processor_estimate(pcm_ch, 4, doa_processor_get_spacing_mm(),
                               hr.fundamental_bin, &doa_res);
        const int16_t doa_deg      = doa_res.valid ? (int16_t)lrintf(doa_res.azimuth_deg) : -1;
        const int16_t doa_elev_deg = (doa_res.valid && doa_res.elevation_deg >= 0.0f)
                                     ? (int16_t)lrintf(doa_res.elevation_deg) : -1;
        const uint8_t active_mics  = (uint8_t)doa_res.active_mics;

        if (rms < RMS_MIN) {
            s_conf_ema *= (1.f - EMA_ALPHA);
            if (alarm) {
                sustain_off++;
                if (sustain_off >= SUSTAIN_FRAMES_OFF) {
                    alarm = false;
                    last_pushed_doa  = -1;
                    last_pushed_elev = -1;
                    ESP_LOGI(TAG, "CLEAR (silent)");
                    push_event(DRONE_EVENT_CLEAR, 0.f, 0, rms, -1, -1, active_mics, 0.f);
                }
            }
        } else {
            if (harm_ok) {
                s_conf_ema = EMA_ALPHA * hr.confidence + (1.f - EMA_ALPHA) * s_conf_ema;
            } else {
                s_conf_ema = EMA_ALPHA * fminf(hr.confidence, 0.15f) + (1.f - EMA_ALPHA) * s_conf_ema;
            }

            const int active_metric = harm_ok ? 1 : 0;
            const int sens_lvl = audio_processor_get_sensitivity();
            const float conf_on_thresh = (sens_lvl >= 5) ? 0.20f :
                                         (sens_lvl == 4) ? 0.24f : CONF_ON;
            const int sustain_on_req   = (sens_lvl >= 4) ? 2 : SUSTAIN_FRAMES_ON;

            if (!alarm) {
                if (harm_ok && s_conf_ema >= conf_on_thresh) {
                    sustain_on++;
                    sustain_off = 0;
                    if (sustain_on >= sustain_on_req) {
                        alarm = true;
                        detection_count++;
                        last_pushed_doa  = doa_deg;
                        last_pushed_elev = doa_elev_deg;
                        last_push_us     = now_us;
                        ESP_LOGW(TAG, "ALARM #%d: f0=%.1f Hz conf=%.2f rms=%.5f mics=%u Az=%d deg (%s) Elev=%d deg",
                                 detection_count, hr.fundamental_hz, (double)s_conf_ema, rms,
                                 (unsigned)active_mics, (int)doa_deg,
                                 doa_angle_to_cardinal(doa_deg, active_mics), (int)doa_elev_deg);
                        push_event(DRONE_EVENT_ALARM, hr.confidence,
                                   hr.fundamental_bin > 255 ? 255 : hr.fundamental_bin, rms,
                                   doa_deg, doa_elev_deg, active_mics, doa_res.confidence);
                    }
                } else {
                    sustain_on = 0;
                }
            } else {
                if (!harm_ok || s_conf_ema < CONF_OFF) {
                    sustain_off++;
                    sustain_on = 0;
                    if (sustain_off >= SUSTAIN_FRAMES_OFF) {
                        alarm = false;
                        last_pushed_doa  = -1;
                        last_pushed_elev = -1;
                        ESP_LOGI(TAG, "CLEAR (harm_ok=%d conf=%.2f)", harm_ok, (double)s_conf_ema);
                        push_event(DRONE_EVENT_CLEAR, hr.confidence, active_metric, rms,
                                   -1, -1, active_mics, 0.f);
                    }
                } else {
                    sustain_off = 0;
                    /* Live DoA/Elevation tracking during active alarm */
                    if (doa_res.valid && (now_us - last_push_us) >= 600000LL &&
                        (last_pushed_doa < 0 ||
                         abs((int)doa_deg - (int)last_pushed_doa) >= 8 ||
                         abs((int)doa_elev_deg - (int)last_pushed_elev) >= 8)) {
                        last_pushed_doa  = doa_deg;
                        last_pushed_elev = doa_elev_deg;
                        last_push_us     = now_us;
                        push_event(DRONE_EVENT_ALARM, hr.confidence,
                                   hr.fundamental_bin > 255 ? 255 : hr.fundamental_bin, rms,
                                   doa_deg, doa_elev_deg, active_mics, doa_res.confidence);
                    }
                }
            }
        }

        static int64_t last_cal_us = 0;
        if (alarm && now_us - last_cal_us > 1000000LL) {
            ESP_LOGI(TAG,
                     "cal: f0=%.1f Hz h2=%.2f h3=%.2f snr=%.1f conf=%.2f mics=%u Az=%d deg (%s) Elev=%d deg",
                     hr.fundamental_hz, hr.h2_ratio, hr.h3_ratio, hr.snr,
                     (double)conf_display, (unsigned)active_mics,
                     (int)doa_deg, doa_angle_to_cardinal(doa_deg, active_mics), (int)doa_elev_deg);
            last_cal_us = now_us;
        }

#if CONFIG_BATEAR_AUDIO_DEBUG_LOG
        static int64_t last_dbg_us = 0;
        if (audio_processor_get_debug_log() && now_us - last_dbg_us > 1000000LL) {
            ESP_LOGI(TAG,
                     "dbg: mics=%u rms[N=%.4f Z=%.4f O=%.4f W=%.4f] conf=%.2f "
                     "f0=%.1f snr=%.1f Az=%d deg (%s) Elev=%d deg alarm=%d",
                     (unsigned)active_mics,
                     doa_res.rms_ch[0], doa_res.rms_ch[1], doa_res.rms_ch[2], doa_res.rms_ch[3],
                     (double)s_conf_ema, hr.fundamental_hz, hr.snr,
                     (int)doa_deg, doa_angle_to_cardinal(doa_deg, active_mics),
                     (int)doa_elev_deg, alarm ? 1 : 0);
            last_dbg_us = now_us;
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(HOP_MS));
    }
}
