/*
 * doa_processor.c — Frequency-domain SRP-PHAT Direction of Arrival (DoA)
 */

#include "doa_processor.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "doa";

/* Speed of sound at ~20 °C (m/s) */
#define SPEED_OF_SOUND_MPS  343.0f

/* Minimum normalized AC RMS on each microphone channel to consider it connected */
#define DOA_MIN_CHAN_RMS    0.00008f

/* Number of frequency bins around f0, 2*f0, 3*f0 used for phase correlation */
#define DOA_HARM_HALF_WIDTH 2
#define DOA_MAX_ACTIVE_BINS 24

/* Circular EMA smoothing factor for azimuth */
#define DOA_EMA_ALPHA       0.40f

/* Working FFT buffers for up to 4 channels (interleaved complex float: 2 * 1024 floats each) */
static float __attribute__((aligned(16))) s_fft_ch[DOA_MAX_MICS][2 * AUDIO_PROC_FFT_SIZE];

static bool  s_ema_init       = false;
static float s_ema_sin        = 0.0f;
static float s_ema_cos        = 1.0f;
static float s_mic_spacing_mm = 83.0f; /* Default: 83 mm (8.3 cm) */

esp_err_t doa_processor_init(void)
{
    s_ema_init = false;
    s_ema_sin  = 0.0f;
    s_ema_cos  = 1.0f;
    ESP_LOGI(TAG, "DoA processor initialized (spacing=%.0f mm)", (double)s_mic_spacing_mm);
    return ESP_OK;
}

void doa_processor_deinit(void)
{
    s_ema_init = false;
}

void doa_processor_set_spacing_mm(float spacing_mm)
{
    if (spacing_mm < 20.0f) spacing_mm = 20.0f;
    if (spacing_mm > 500.0f) spacing_mm = 500.0f;
    s_mic_spacing_mm = spacing_mm;
    ESP_LOGI(TAG, "Mic array spacing set to %.0f mm", (double)s_mic_spacing_mm);
}

float doa_processor_get_spacing_mm(void)
{
    return s_mic_spacing_mm;
}

const char *doa_angle_to_cardinal(int16_t deg)
{
    if (deg < 0 || deg > 360) {
        return "N/A";
    }
    if (deg <= 180) {
        if (deg < 30)  return "LINKS";
        if (deg < 70)  return "LINKS-VOOR";
        if (deg <= 110) return "MIDDEN";
        if (deg <= 150) return "RECHTS-VOOR";
        return "RECHTS";
    }
    static const char * const DIRS[8] = {
        "N", "NE", "E", "SE", "S", "SW", "W", "NW"
    };
    int idx = ((deg + 22) % 360) / 45;
    return DIRS[idx & 7];
}

static float prepare_channel_fft(float *fft_io, const int32_t *pcm, int n_samples)
{
    const float inv_full_scale = 1.f / 2147483648.f;
    const float *win = audio_processor_window();
    const float gain = audio_processor_get_gain();

    double dc_sum = 0.0;
    for (int i = 0; i < n_samples; i++) {
        dc_sum += (double)pcm[i] * (double)inv_full_scale;
    }
    const float dc_mean = (float)(dc_sum / (double)n_samples);

    double sum_sq = 0.0;
    for (int i = 0; i < n_samples; i++) {
        float x = ((float)pcm[i] * inv_full_scale - dc_mean) * gain;
        if (x > 1.0f) x = 1.0f;
        if (x < -1.0f) x = -1.0f;
        const float wx = x * win[i];
        fft_io[2 * i]     = wx;
        fft_io[2 * i + 1] = 0.f;
        sum_sq += (double)wx * (double)wx;
    }
    return (float)sqrt(sum_sq / (double)n_samples);
}

static int select_active_bins(int f0_bin, int bins_out[DOA_MAX_ACTIVE_BINS])
{
    int count = 0;
    const int max_bin = AUDIO_PROC_PSD_BINS - 2;

    if (f0_bin > 2 && f0_bin < max_bin) {
        const int harmonics[3] = { f0_bin, f0_bin * 2, f0_bin * 3 };
        for (int h = 0; h < 3; h++) {
            int center = harmonics[h];
            if (center >= max_bin) {
                continue;
            }
            for (int dk = -DOA_HARM_HALF_WIDTH; dk <= DOA_HARM_HALF_WIDTH; dk++) {
                int k = center + dk;
                if (k >= 4 && k < max_bin && count < DOA_MAX_ACTIVE_BINS) {
                    bins_out[count++] = k;
                }
            }
        }
    }

    if (count == 0) {
        const float *psd = audio_processor_last_psd();
        int k_min = (int)(110.0f / AUDIO_PROC_BIN_HZ);
        int k_max = (int)(1500.0f / AUDIO_PROC_BIN_HZ);
        int step  = (k_max - k_min) / 12;
        if (step < 1) step = 1;
        for (int k = k_min; k <= k_max && count < DOA_MAX_ACTIVE_BINS; k += step) {
            if (psd == NULL || psd[k] > 0.0f) {
                bins_out[count++] = k;
            }
        }
    }

    return count;
}

esp_err_t doa_processor_estimate(const int32_t * const pcm_ch[],
                                 int num_mics,
                                 float mic_spacing_mm,
                                 int f0_bin,
                                 DoaResult *out)
{
    if (pcm_ch == NULL || out == NULL || num_mics < 2) {
        return ESP_ERR_INVALID_ARG;
    }
    if (num_mics > DOA_MAX_MICS) {
        num_mics = DOA_MAX_MICS;
    }

    memset(out, 0, sizeof(*out));

    float ch_rms[DOA_MAX_MICS] = { 0 };
    int valid_mics = 0;
    for (int m = 0; m < num_mics; m++) {
        if (pcm_ch[m] == NULL) {
            break;
        }
        ch_rms[m] = prepare_channel_fft(s_fft_ch[m], pcm_ch[m], AUDIO_PROC_FFT_SIZE);
        if (ch_rms[m] < DOA_MIN_CHAN_RMS) {
            break;
        }
        if (audio_processor_fft_forward(s_fft_ch[m]) != ESP_OK) {
            break;
        }
        valid_mics++;
    }

    out->rms_left  = ch_rms[0];
    out->rms_right = ch_rms[1];

    if (valid_mics < 2) {
        out->valid       = false;
        out->active_mics = valid_mics;
        out->azimuth_deg = (float)DOA_INVALID_DEG;
        return ESP_OK;
    }

    const int active_mics = (valid_mics >= 4) ? 4 : 2;
    out->active_mics = active_mics;

    const float spacing = (mic_spacing_mm > 10.0f) ? mic_spacing_mm : s_mic_spacing_mm;
    const float d_m = spacing * 0.001f;
    const float r_m = 0.5f * d_m;

    float mic_x[DOA_MAX_MICS] = { 0 };
    float mic_y[DOA_MAX_MICS] = { 0 };

    if (active_mics == 4) {
        mic_x[0] = +r_m; mic_y[0] =  0.0f;
        mic_x[1] =  0.0f; mic_y[1] = +r_m;
        mic_x[2] = -r_m; mic_y[2] =  0.0f;
        mic_x[3] =  0.0f; mic_y[3] = -r_m;
    } else {
        mic_x[0] =  0.0f; mic_y[0] = -r_m; /* Mic 0 (SEL=GND): Left / 0°   */
        mic_x[1] =  0.0f; mic_y[1] = +r_m; /* Mic 1 (SEL=3V3): Right / 180° */
    }

    int bins[DOA_MAX_ACTIVE_BINS];
    const int n_bins = select_active_bins(f0_bin, bins);
    if (n_bins <= 0) {
        out->valid = false;
        return ESP_OK;
    }

    float pair_dx[6], pair_dy[6];
    float phat_re[6][DOA_MAX_ACTIVE_BINS];
    float phat_im[6][DOA_MAX_ACTIVE_BINS];
    int   n_pairs = 0;

    for (int p = 0; p < active_mics; p++) {
        for (int q = p + 1; q < active_mics; q++) {
            pair_dx[n_pairs] = mic_x[p] - mic_x[q];
            pair_dy[n_pairs] = mic_y[p] - mic_y[q];

            for (int b = 0; b < n_bins; b++) {
                const int k = bins[b];
                const float xp_re = s_fft_ch[p][2 * k];
                const float xp_im = s_fft_ch[p][2 * k + 1];
                const float xq_re = s_fft_ch[q][2 * k];
                const float xq_im = s_fft_ch[q][2 * k + 1];

                const float g_re = xp_re * xq_re + xp_im * xq_im;
                const float g_im = xp_im * xq_re - xp_re * xq_im;
                const float mag  = sqrtf(g_re * g_re + g_im * g_im) + 1e-12f;

                phat_re[n_pairs][b] = g_re / mag;
                phat_im[n_pairs][b] = g_im / mag;
            }
            n_pairs++;
        }
    }

    const int max_angle_deg = (active_mics == 4) ? 355 : 180;
    const int coarse_step   = 5;

    float best_score = -1e9f;
    int   best_deg   = 90;

    for (int deg = 0; deg <= max_angle_deg; deg += coarse_step) {
        const float rad = (float)deg * ((float)M_PI / 180.0f);
        const float ux  = (active_mics == 4) ? cosf(rad) : sinf(rad);
        const float uy  = (active_mics == 4) ? sinf(rad) : -cosf(rad);

        float score = 0.0f;
        for (int pr = 0; pr < n_pairs; pr++) {
            const float tau_pq = (pair_dx[pr] * ux + pair_dy[pr] * uy) / SPEED_OF_SOUND_MPS;
            for (int b = 0; b < n_bins; b++) {
                const float freq_hz = (float)bins[b] * AUDIO_PROC_BIN_HZ;
                const float phase   = 2.0f * (float)M_PI * freq_hz * tau_pq;
                score += phat_re[pr][b] * cosf(phase) + phat_im[pr][b] * sinf(phase);
            }
        }

        if (score > best_score) {
            best_score = score;
            best_deg   = deg;
        }
    }

    int fine_best_deg = best_deg;
    for (int d = best_deg - 4; d <= best_deg + 4; d++) {
        int deg = d;
        if (active_mics == 4) {
            deg = (deg + 360) % 360;
        } else {
            if (deg < 0 || deg > 180) continue;
        }

        const float rad = (float)deg * ((float)M_PI / 180.0f);
        const float ux  = (active_mics == 4) ? cosf(rad) : sinf(rad);
        const float uy  = (active_mics == 4) ? sinf(rad) : -cosf(rad);

        float score = 0.0f;
        for (int pr = 0; pr < n_pairs; pr++) {
            const float tau_pq = (pair_dx[pr] * ux + pair_dy[pr] * uy) / SPEED_OF_SOUND_MPS;
            for (int b = 0; b < n_bins; b++) {
                const float freq_hz = (float)bins[b] * AUDIO_PROC_BIN_HZ;
                const float phase   = 2.0f * (float)M_PI * freq_hz * tau_pq;
                score += phat_re[pr][b] * cosf(phase) + phat_im[pr][b] * sinf(phase);
            }
        }
        if (score > best_score) {
            best_score    = score;
            fine_best_deg = deg;
        }
    }

    const float max_possible = (float)(n_pairs * n_bins);
    float coherence = (max_possible > 0.0f) ? (best_score / max_possible) : 0.0f;
    if (coherence < 0.0f) coherence = 0.0f;
    if (coherence > 1.0f) coherence = 1.0f;

    const float est_rad = (float)fine_best_deg * ((float)M_PI / 180.0f);
    if (!s_ema_init) {
        s_ema_sin  = sinf(est_rad);
        s_ema_cos  = cosf(est_rad);
        s_ema_init = true;
    } else {
        s_ema_sin = DOA_EMA_ALPHA * sinf(est_rad) + (1.0f - DOA_EMA_ALPHA) * s_ema_sin;
        s_ema_cos = DOA_EMA_ALPHA * cosf(est_rad) + (1.0f - DOA_EMA_ALPHA) * s_ema_cos;
    }

    float smooth_deg = atan2f(s_ema_sin, s_ema_cos) * (180.0f / (float)M_PI);
    if (smooth_deg < 0.0f) {
        smooth_deg += 360.0f;
    }
    if (active_mics == 2 && smooth_deg > 180.0f) {
        smooth_deg = (smooth_deg > 270.0f) ? 0.0f : 180.0f;
    }

    out->valid       = true;
    out->azimuth_deg = smooth_deg;
    out->confidence  = coherence;
    return ESP_OK;
}

void doa_simulate_multichannel_pcm(int32_t *pcm_ch[],
                                   int num_mics,
                                   int n_samples,
                                   float mic_spacing_mm,
                                   float f0_hz,
                                   float azimuth_deg)
{
    const float d_m = (mic_spacing_mm > 10.0f ? mic_spacing_mm : 100.0f) * 0.001f;
    const float r_m = 0.5f * d_m;

    float mic_x[DOA_MAX_MICS] = { +r_m, 0.0f, -r_m, 0.0f };
    float mic_y[DOA_MAX_MICS] = { 0.0f, +r_m, 0.0f, -r_m };
    if (num_mics == 2) {
        mic_x[0] = 0.0f; mic_y[0] = -r_m;
        mic_x[1] = 0.0f; mic_y[1] = +r_m;
    }

    const float rad = azimuth_deg * ((float)M_PI / 180.0f);
    const float ux  = (num_mics == 4) ? cosf(rad) : sinf(rad);
    const float uy  = (num_mics == 4) ? sinf(rad) : -cosf(rad);

    const float amp = 0.05f * 2147483647.0f;
    const float fs  = (float)AUDIO_PROC_SAMPLE_RATE_HZ;

    for (int m = 0; m < num_mics && m < DOA_MAX_MICS; m++) {
        if (pcm_ch[m] == NULL) continue;
        const float tau_m = (mic_x[m] * ux + mic_y[m] * uy) / SPEED_OF_SOUND_MPS;

        for (int n = 0; n < n_samples; n++) {
            const float t = ((float)n / fs) + tau_m;
            const float sig = 1.00f * sinf(2.0f * (float)M_PI * f0_hz * t)
                            + 0.45f * sinf(2.0f * (float)M_PI * (2.0f * f0_hz) * t)
                            + 0.30f * sinf(2.0f * (float)M_PI * (3.0f * f0_hz) * t);
            pcm_ch[m][n] = (int32_t)(amp * sig);
        }
    }
}
