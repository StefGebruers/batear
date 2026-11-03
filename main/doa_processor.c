/*
 * doa_processor.c — 2D/3D Frequency-domain SNR-weighted SRP-PHAT DoA estimator
 *
 * Computes Azimuth (0..360 deg) and Elevation (0..90 deg) for a 4-microphone
 * planar cross array ("kruisverband"), with automatic fallback to 1D Azimuth
 * (0..180 deg) if only 2 microphones are connected.
 */

#include "doa_processor.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "doa";

/* Speed of sound at ~20 °C (m/s) */
#define SPEED_OF_SOUND_MPS  343.0f

/* Minimum normalized AC RMS on each microphone channel to consider it connected */
#define DOA_MIN_CHAN_RMS    0.00008f

/* Number of frequency bins around harmonics and broadband peaks */
#define DOA_HARM_HALF_WIDTH 2
#define DOA_MAX_ACTIVE_BINS 36

/* Number of precomputed delay points per microphone pair in the 1D GCC-PHAT LUT */
#define DOA_TAU_LUT_SIZE    121

/* Base EMA smoothing factor for azimuth and elevation (scaled by coherence) */
#define DOA_EMA_ALPHA_BASE  0.42f

/* Working FFT buffers for up to 4 channels (interleaved complex float: 2 * 1024 floats each) */
static float __attribute__((aligned(16))) s_fft_ch[DOA_MAX_MICS][2 * AUDIO_PROC_FFT_SIZE];

/* Per-channel harmonic tonal quality weight (0.15 .. 1.0) so clean mics dominate over noisy wires */
static float s_ch_quality[DOA_MAX_MICS];

/* Precomputed 1D GCC-PHAT curve per microphone pair for ultra-fast 2D (Azimuth x Elevation) search */
static float s_gcc_lut[6][DOA_TAU_LUT_SIZE];

static bool  s_ema_init       = false;
static float s_ema_sin        = 0.0f;
static float s_ema_cos        = 1.0f;
static float s_ema_elev_deg   = 0.0f;
static float s_mic_spacing_mm = 83.0f; /* Default: 83 mm (distance between opposite mics N-Z and O-W) */
static float s_north_offset_deg = 0.0f; /* True North offset in degrees: true_az = (meas_az + offset) % 360 */

esp_err_t doa_processor_init(void)
{
    s_ema_init     = false;
    s_ema_sin      = 0.0f;
    s_ema_cos      = 1.0f;
    s_ema_elev_deg = 0.0f;

    nvs_handle_t h;
    if (nvs_open("lora_cfg", NVS_READONLY, &h) == ESP_OK) {
        uint16_t off = 0;
        if (nvs_get_u16(h, "north_offset", &off) == ESP_OK) {
            s_north_offset_deg = (float)(off % 360);
        }
        nvs_close(h);
    }

    ESP_LOGI(TAG, "DoA processor initialized (spacing=%.0f mm, north_offset=%.0f deg)",
             (double)s_mic_spacing_mm, (double)s_north_offset_deg);
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

void doa_processor_set_north_offset_deg(float offset_deg)
{
    while (offset_deg >= 360.0f) offset_deg -= 360.0f;
    while (offset_deg < 0.0f)    offset_deg += 360.0f;
    s_north_offset_deg = offset_deg;
    ESP_LOGI(TAG, "True North offset set to %.0f deg", (double)s_north_offset_deg);
}

float doa_processor_get_north_offset_deg(void)
{
    return s_north_offset_deg;
}

const char *doa_angle_to_cardinal(int16_t deg, int active_mics)
{
    if (deg < 0 || deg > 360) {
        return "N/A";
    }
    if (active_mics < 4) {
        if (deg < 30)   return "LINKS";
        if (deg < 70)   return "LINKS-VOOR";
        if (deg <= 110) return "MIDDEN";
        if (deg <= 150) return "RECHTS-VOOR";
        return "RECHTS";
    }
    /* 4-mic 360° Dutch compass labels:
     *   0° = Noord (Voor), 90° = Oost (Rechts), 180° = Zuid (Achter), 270° = West (Links)
     */
    static const char * const DIRS_NL[8] = {
        "N", "NO", "O", "ZO", "Z", "ZW", "W", "NW"
    };
    int idx = (((int)deg + 22) % 360) / 45;
    return DIRS_NL[idx & 7];
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

/**
 * Select up to DOA_MAX_ACTIVE_BINS bins combining:
 *   1. The detected harmonic series (f0, 2*f0, 3*f0, 4*f0) with a 1.5x priority weight.
 *   2. High-SNR spectral peaks across the 140..1900 Hz band so broadband propeller
 *      energy averages out single-frequency room reflections (multipath).
 */
static int select_active_bins_weighted(int f0_bin,
                                       int active_mics,
                                       int bins_out[DOA_MAX_ACTIVE_BINS],
                                       float weights_out[DOA_MAX_ACTIVE_BINS])
{
    int count = 0;
    const int max_bin = (int)(1950.0f / AUDIO_PROC_BIN_HZ);
    const int min_bin = (int)(120.0f / AUDIO_PROC_BIN_HZ);

    /* Helper: compute mean spectral magnitude across active mics at bin k */
    if (f0_bin >= min_bin && f0_bin < max_bin) {
        const int harmonics[4] = { f0_bin, f0_bin * 2, f0_bin * 3, f0_bin * 4 };
        for (int h = 0; h < 4; h++) {
            int center = harmonics[h];
            if (center >= max_bin) continue;
            for (int dk = -DOA_HARM_HALF_WIDTH; dk <= DOA_HARM_HALF_WIDTH; dk++) {
                int k = center + dk;
                if (k < min_bin || k >= max_bin || count >= DOA_MAX_ACTIVE_BINS) continue;
                float mag_sum = 0.0f;
                for (int m = 0; m < active_mics; m++) {
                    float re = s_fft_ch[m][2 * k];
                    float im = s_fft_ch[m][2 * k + 1];
                    mag_sum += sqrtf(re * re + im * im);
                }
                bins_out[count]    = k;
                weights_out[count] = 1.5f * sqrtf(mag_sum / (float)active_mics + 1e-12f);
                count++;
            }
        }
    }

    /* Fill remaining slots with the strongest spectral bins across 140..1850 Hz
     * (1 bin per ~80 Hz sub-band) so multipath interference averages out. */
    const int band_step = 5; /* ~78 Hz per sub-band */
    for (int k_start = min_bin; k_start + band_step < max_bin && count < DOA_MAX_ACTIVE_BINS; k_start += band_step) {
        int   best_k   = -1;
        float best_mag = 0.0f;
        float min_mag  = 1e30f;

        for (int k = k_start; k < k_start + band_step; k++) {
            /* Skip if already added in harmonic pass */
            bool dup = false;
            for (int i = 0; i < count; i++) {
                if (bins_out[i] == k) {
                    dup = true;
                    break;
                }
            }
            if (dup) continue;

            float mag_sum = 0.0f;
            for (int m = 0; m < active_mics; m++) {
                float re = s_fft_ch[m][2 * k];
                float im = s_fft_ch[m][2 * k + 1];
                mag_sum += sqrtf(re * re + im * im);
            }
            if (mag_sum < min_mag) min_mag = mag_sum;
            if (mag_sum > best_mag) {
                best_mag = mag_sum;
                best_k   = k;
            }
        }

        /* Only include sub-band peak if it stands out above its local floor */
        if (best_k >= min_bin && best_mag > 1.8f * (min_mag + 1e-9f)) {
            bins_out[count]    = best_k;
            weights_out[count] = sqrtf(best_mag / (float)active_mics);
            count++;
        }
    }

    /* Normalize weights so sum(weights) == 1.0 */
    float w_sum = 0.0f;
    for (int i = 0; i < count; i++) {
        w_sum += weights_out[i];
    }
    if (w_sum > 1e-12f) {
        for (int i = 0; i < count; i++) {
            weights_out[i] /= w_sum;
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
    out->azimuth_deg   = (float)DOA_INVALID_DEG;
    out->elevation_deg = (float)DOA_INVALID_DEG;

    int valid_mics = 0;
    for (int m = 0; m < num_mics; m++) {
        if (pcm_ch[m] == NULL) {
            continue;
        }
        out->rms_ch[m] = prepare_channel_fft(s_fft_ch[m], pcm_ch[m], AUDIO_PROC_FFT_SIZE);
        if (out->rms_ch[m] >= DOA_MIN_CHAN_RMS) {
            if (audio_processor_fft_forward(s_fft_ch[m]) == ESP_OK && valid_mics == m) {
                valid_mics++;
            }
        }
    }

    out->rms_left  = out->rms_ch[0];
    out->rms_right = out->rms_ch[1];

    if (valid_mics < 2) {
        out->valid       = false;
        out->active_mics = valid_mics;
        return ESP_OK;
    }

    const int active_mics = (valid_mics >= 4) ? 4 : 2;
    out->active_mics = active_mics;

    const float spacing = (mic_spacing_mm > 10.0f) ? mic_spacing_mm : s_mic_spacing_mm;
    const float d_m = spacing * 0.001f;
    const float r_m = 0.5f * d_m;

    /* Compute per-channel tonal SNR quality at f0 (or across 140..1200 Hz) so that
     * if one microphone wire has bus noise, clean microphone pairs dominate GCC-PHAT. */
    for (int m = 0; m < active_mics; m++) {
        float sig_p = 0.0f;
        float tot_p = 1e-12f;
        const int k_min = (int)(120.0f / AUDIO_PROC_BIN_HZ);
        const int k_max = (int)(1800.0f / AUDIO_PROC_BIN_HZ);
        for (int k = k_min; k < k_max; k++) {
            float re = s_fft_ch[m][2 * k];
            float im = s_fft_ch[m][2 * k + 1];
            float p  = re * re + im * im;
            tot_p += p;
            if (f0_bin >= k_min &&
                (abs(k - f0_bin) <= 2 || abs(k - 2 * f0_bin) <= 2 || abs(k - 3 * f0_bin) <= 2)) {
                sig_p += p;
            } else if (f0_bin < k_min && p > sig_p) {
                sig_p = p;
            }
        }
        float ratio = sig_p / tot_p;
        /* Clamp channel quality weight between 0.15 and 1.0 */
        float q = sqrtf(ratio * 8.0f);
        if (q < 0.15f) q = 0.15f;
        if (q > 1.00f) q = 1.00f;
        s_ch_quality[m] = q;
    }

    /* Microphone coordinates in meters:
     * 4-mic cross ("kruisverband"):
     *   Mic 0 (GPIO6, SEL=GND): Noord / Voor   (+r,  0) -> Azimuth 0°
     *   Mic 1 (GPIO6, SEL=3V3): Zuid  / Achter (-r,  0) -> Azimuth 180°
     *   Mic 2 (GPIO7, SEL=GND): Oost  / Rechts ( 0, +r) -> Azimuth 90°
     *   Mic 3 (GPIO7, SEL=3V3): West  / Links  ( 0, -r) -> Azimuth 270°
     */
    float mic_x[DOA_MAX_MICS] = { 0 };
    float mic_y[DOA_MAX_MICS] = { 0 };

    if (active_mics == 4) {
        mic_x[0] = +r_m; mic_y[0] =  0.0f; /* Ch0: Noord (0°)   */
        mic_x[1] = -r_m; mic_y[1] =  0.0f; /* Ch1: Zuid  (180°) */
        mic_x[2] =  0.0f; mic_y[2] = +r_m; /* Ch2: Oost  (90°)  */
        mic_x[3] =  0.0f; mic_y[3] = -r_m; /* Ch3: West  (270°) */
    } else {
        mic_x[0] =  0.0f; mic_y[0] = -r_m; /* Ch0: Links  (0°)   */
        mic_x[1] =  0.0f; mic_y[1] = +r_m; /* Ch1: Rechts (180°) */
    }

    int   bins[DOA_MAX_ACTIVE_BINS];
    float bin_weights[DOA_MAX_ACTIVE_BINS];
    const int n_bins = select_active_bins_weighted(f0_bin, active_mics, bins, bin_weights);
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

                /* Weighted PHAT: unit phase vector scaled by bin SNR weight and pair channel quality */
                const float pair_q = s_ch_quality[p] * s_ch_quality[q];
                phat_re[n_pairs][b] = (g_re / mag) * bin_weights[b] * pair_q;
                phat_im[n_pairs][b] = (g_im / mag) * bin_weights[b] * pair_q;
            }
            n_pairs++;
        }
    }

    /* Step 1: Precompute 1D GCC-PHAT lookup table for each pair over [-tau_max, +tau_max] */
    const float tau_max = (d_m / SPEED_OF_SOUND_MPS) * 1.05f;
    const float inv_tau_max = (tau_max > 1e-9f) ? (1.0f / tau_max) : 1.0f;
    const int   half_lut = (DOA_TAU_LUT_SIZE - 1) / 2;

    for (int pr = 0; pr < n_pairs; pr++) {
        for (int ti = 0; ti < DOA_TAU_LUT_SIZE; ti++) {
            const float tau = ((float)(ti - half_lut) / (float)half_lut) * tau_max;
            float acc = 0.0f;
            for (int b = 0; b < n_bins; b++) {
                const float freq_hz = (float)bins[b] * AUDIO_PROC_BIN_HZ;
                const float phase   = 2.0f * (float)M_PI * freq_hz * tau;
                acc += phat_re[pr][b] * cosf(phase) + phat_im[pr][b] * sinf(phase);
            }
            s_gcc_lut[pr][ti] = acc;
        }
    }

    /* Step 2: 2D search over Azimuth (and Elevation when active_mics == 4) using LUT */
    float best_score    = -1e9f;
    int   best_az_deg   = (active_mics == 4) ? 0 : 90;
    int   best_elev_deg = 0;

    if (active_mics == 4) {
        /* Coarse 2D grid: Azimuth 0..355 (step 5°), Elevation 0..85 (step 5°) */
        for (int el = 0; el <= 85; el += 5) {
            const float cos_el = cosf((float)el * ((float)M_PI / 180.0f));
            for (int az = 0; az < 360; az += 5) {
                const float rad = (float)az * ((float)M_PI / 180.0f);
                const float ux  = cosf(rad) * cos_el;
                const float uy  = sinf(rad) * cos_el;

                float score = 0.0f;
                for (int pr = 0; pr < n_pairs; pr++) {
                    const float tau_pq = (pair_dx[pr] * ux + pair_dy[pr] * uy) / SPEED_OF_SOUND_MPS;
                    float f_idx = (tau_pq * inv_tau_max) * (float)half_lut + (float)half_lut;
                    int idx = (int)(f_idx + 0.5f);
                    if (idx < 0) idx = 0;
                    if (idx >= DOA_TAU_LUT_SIZE) idx = DOA_TAU_LUT_SIZE - 1;
                    score += s_gcc_lut[pr][idx];
                }
                if (score > best_score) {
                    best_score    = score;
                    best_az_deg   = az;
                    best_elev_deg = el;
                }
            }
        }

        /* Fine 2D refinement (1° steps in Azimuth and Elevation around coarse winner) */
        const int c_az = best_az_deg;
        const int c_el = best_elev_deg;
        for (int el = c_el - 4; el <= c_el + 4; el++) {
            if (el < 0 || el > 90) continue;
            const float cos_el = cosf((float)el * ((float)M_PI / 180.0f));
            for (int d_az = c_az - 4; d_az <= c_az + 4; d_az++) {
                const int az = (d_az + 360) % 360;
                const float rad = (float)az * ((float)M_PI / 180.0f);
                const float ux  = cosf(rad) * cos_el;
                const float uy  = sinf(rad) * cos_el;

                float score = 0.0f;
                for (int pr = 0; pr < n_pairs; pr++) {
                    const float tau_pq = (pair_dx[pr] * ux + pair_dy[pr] * uy) / SPEED_OF_SOUND_MPS;
                    float f_idx = (tau_pq * inv_tau_max) * (float)half_lut + (float)half_lut;
                    int idx = (int)(f_idx + 0.5f);
                    if (idx < 0) idx = 0;
                    if (idx >= DOA_TAU_LUT_SIZE) idx = DOA_TAU_LUT_SIZE - 1;
                    score += s_gcc_lut[pr][idx];
                }
                if (score > best_score) {
                    best_score    = score;
                    best_az_deg   = az;
                    best_elev_deg = el;
                }
            }
        }
    } else {
        /* 2-mic 1D search: 0..180 deg at 1° resolution */
        for (int az = 0; az <= 180; az++) {
            const float rad = (float)az * ((float)M_PI / 180.0f);
            const float ux  = sinf(rad);
            const float uy  = -cosf(rad);
            const float tau_pq = (pair_dx[0] * ux + pair_dy[0] * uy) / SPEED_OF_SOUND_MPS;
            float f_idx = (tau_pq * inv_tau_max) * (float)half_lut + (float)half_lut;
            int idx = (int)(f_idx + 0.5f);
            if (idx < 0) idx = 0;
            if (idx >= DOA_TAU_LUT_SIZE) idx = DOA_TAU_LUT_SIZE - 1;
            const float score = s_gcc_lut[0][idx];
            if (score > best_score) {
                best_score  = score;
                best_az_deg = az;
            }
        }
    }

    float coherence = (n_pairs > 0) ? (best_score / (float)n_pairs) : 0.0f;
    if (coherence < 0.0f) coherence = 0.0f;
    if (coherence > 1.0f) coherence = 1.0f;

    /* Coherence-weighted EMA smoothing so reflections with low coherence don't jerk the angle */
    float alpha = DOA_EMA_ALPHA_BASE * (0.35f + 0.65f * coherence);
    if (alpha > 0.65f) alpha = 0.65f;
    if (alpha < 0.12f) alpha = 0.12f;

    const float est_rad = (float)best_az_deg * ((float)M_PI / 180.0f);
    if (!s_ema_init) {
        s_ema_sin      = sinf(est_rad);
        s_ema_cos      = cosf(est_rad);
        s_ema_elev_deg = (float)best_elev_deg;
        s_ema_init     = true;
    } else {
        /* At very high elevation (>80°), azimuth becomes geometric zenith; freeze azimuth drift */
        if (active_mics < 4 || best_elev_deg <= 82) {
            s_ema_sin = alpha * sinf(est_rad) + (1.0f - alpha) * s_ema_sin;
            s_ema_cos = alpha * cosf(est_rad) + (1.0f - alpha) * s_ema_cos;
        }
        s_ema_elev_deg = alpha * (float)best_elev_deg + (1.0f - alpha) * s_ema_elev_deg;
    }

    float smooth_az = atan2f(s_ema_sin, s_ema_cos) * (180.0f / (float)M_PI);
    if (smooth_az < 0.0f) {
        smooth_az += 360.0f;
    }
    if (active_mics == 2 && smooth_az > 180.0f) {
        smooth_az = (smooth_az > 270.0f) ? 0.0f : 180.0f;
    }

    /* Apply true North alignment offset */
    float true_az = smooth_az + s_north_offset_deg;
    while (true_az >= 360.0f) true_az -= 360.0f;
    while (true_az < 0.0f)    true_az += 360.0f;

    out->valid         = true;
    out->azimuth_deg   = true_az;
    out->elevation_deg = (active_mics == 4) ? s_ema_elev_deg : (float)DOA_INVALID_DEG;
    out->confidence    = coherence;
    return ESP_OK;
}

void doa_simulate_multichannel_pcm(int32_t *pcm_ch[],
                                   int num_mics,
                                   int n_samples,
                                   float mic_spacing_mm,
                                   float f0_hz,
                                   float azimuth_deg)
{
    const float d_m = (mic_spacing_mm > 10.0f ? mic_spacing_mm : 83.0f) * 0.001f;
    const float r_m = 0.5f * d_m;

    float mic_x[DOA_MAX_MICS] = { +r_m, -r_m,  0.0f,  0.0f };
    float mic_y[DOA_MAX_MICS] = { 0.0f,  0.0f, +r_m, -r_m };
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
