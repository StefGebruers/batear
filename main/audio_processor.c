/*
 * audio_processor.c — FFT + Hanning + PSD + harmonic analysis (ESP-DSP)
 */

#include "audio_processor.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"
#include "dsps_fft2r.h"
#include "esp_log.h"

static const char *TAG = "audio_proc";

static float __attribute__((aligned(16))) s_window[AUDIO_PROC_FFT_SIZE];
static float __attribute__((aligned(16))) s_psd[AUDIO_PROC_PSD_BINS];
static float s_last_rms;

/* Runtime-tunable sensitivity (1=Strict .. 5=Max, default 3=Balanced)
 * and digital mic gain (default 4.0x = +12 dB to align ICS-43434 -38 dBFS with INMP441 -26 dBFS). */
static int   s_sensitivity_level = 3;
static float s_mic_gain          = 4.0f;

typedef struct {
    float min_peak_snr;     /* Minimum fundamental SNR above noise floor */
    float min_h2_ratio;     /* Minimum P(2*f0)/P(f0) */
    float min_h3_ratio;     /* Minimum P(3*f0)/P(f0) */
    float min_harm_snr;     /* Minimum SNR of 2*f0 and 3*f0 above noise floor */
    float min_valley_ratio; /* Anti-false-alarm: harmonic peak vs inter-harmonic valley (1.5*f0, 2.5*f0) */
    float snr_norm_scale;   /* Confidence normalization scale */
} SensitivityProfile;

/* Profiles 1..5:
 * - Level 1 (Strict):      Near original BatEar thresholds, only loud close sources
 * - Level 2 (Conservative): Moderate distance, very high noise immunity
 * - Level 3 (Balanced):    Default — good room-scale sensitivity + valley broadband rejection
 * - Level 4 (High):        High sensitivity for quieter/distant drones
 * - Level 5 (Max):         Maximum sensitivity for long-range testing
 */
static const SensitivityProfile s_profiles[5] = {
    /* 1: Strict */       { 8.0f, 0.050f, 0.025f, 4.0f, 4.0f, 18.0f },
    /* 2: Conservative */ { 6.0f, 0.020f, 0.008f, 3.0f, 3.2f, 12.0f },
    /* 3: Balanced */     { 4.5f, 0.008f, 0.003f, 2.5f, 2.5f,  8.0f },
    /* 4: High */         { 3.5f, 0.004f, 0.0015f, 2.0f, 2.0f, 6.0f },
    /* 5: Max */          { 2.5f, 0.002f, 0.0008f, 1.6f, 1.6f, 4.5f },
};

void audio_processor_set_sensitivity(int level)
{
    if (level < 1) level = 1;
    if (level > 5) level = 5;
    s_sensitivity_level = level;
    ESP_LOGI(TAG, "Sensitivity set to level %d/5", s_sensitivity_level);
}

int audio_processor_get_sensitivity(void)
{
    return s_sensitivity_level;
}

void audio_processor_set_gain(float gain)
{
    if (gain < 0.5f) gain = 0.5f;
    if (gain > 32.0f) gain = 32.0f;
    s_mic_gain = gain;
    ESP_LOGI(TAG, "Digital mic gain set to %.1fx (+%.1f dB)",
             (double)s_mic_gain, 20.0 * log10((double)s_mic_gain));
}

float audio_processor_get_gain(void)
{
    return s_mic_gain;
}

/* --- Default weak noise floor: mean linear power above 75 Hz (bin 5) --- */
__attribute__((weak)) float audio_processor_noise_floor_estimate(const float *psd, int n_bins_half)
{
    if (n_bins_half < 8 || psd == NULL) {
        return 1e-18f;
    }
    const int start_bin = 5; /* Skip DC & sub-75 Hz rumble */
    double acc = 0.0;
    for (int i = start_bin; i < n_bins_half; i++) {
        acc += (double)psd[i];
    }
    float nf = (float)(acc / (double)(n_bins_half - start_bin));
    return (nf < 1e-18f) ? 1e-18f : nf;
}

static void build_hanning(void)
{
    const int n = AUDIO_PROC_FFT_SIZE;
    if (n == 1) {
        s_window[0] = 1.f;
        return;
    }
    const float scale = 2.f * (float)M_PI / (float)(n - 1);
    for (int i = 0; i < n; i++) {
        s_window[i] = 0.5f * (1.f - cosf(scale * (float)i));
    }
}

esp_err_t audio_processor_init(void)
{
    build_hanning();
    esp_err_t err = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "dsps_fft2r_init_fc32 failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "FFT init OK (max=%d, N=%d, sens=%d, gain=%.1fx)",
             CONFIG_DSP_MAX_FFT_SIZE, AUDIO_PROC_FFT_SIZE, s_sensitivity_level, (double)s_mic_gain);
    return ESP_OK;
}

void audio_processor_deinit(void)
{
    dsps_fft2r_deinit_fc32();
}

// cppcheck-suppress unusedFunction
const float *audio_processor_window(void)
{
    return s_window;
}

float audio_processor_prepare_fft_input(float *fft_io, const int32_t *pcm, int n_samples)
{
    if (fft_io == NULL || pcm == NULL || n_samples != AUDIO_PROC_FFT_SIZE) {
        return 0.f;
    }

    const float inv_full_scale = 1.f / 2147483648.f;

    /* Step 1: Compute frame DC offset (mean) to eliminate MEMS mic DC bias leakage */
    double dc_sum = 0.0;
    for (int i = 0; i < AUDIO_PROC_FFT_SIZE; i++) {
        dc_sum += (double)pcm[i] * (double)inv_full_scale;
    }
    const float dc_mean = (float)(dc_sum / (double)AUDIO_PROC_FFT_SIZE);

    /* Step 2: Subtract DC, apply digital pre-gain, and apply Hanning window */
    double sum_sq = 0.0;
    const float gain = s_mic_gain;

    for (int i = 0; i < AUDIO_PROC_FFT_SIZE; i++) {
        float x = ((float)pcm[i] * inv_full_scale - dc_mean) * gain;
        if (x > 1.0f) x = 1.0f;
        if (x < -1.0f) x = -1.0f;
        const float wx = x * s_window[i];
        fft_io[2 * i]     = wx;
        fft_io[2 * i + 1] = 0.f;
        sum_sq += (double)wx * (double)wx;
    }

    s_last_rms = (float)sqrt(sum_sq / (double)AUDIO_PROC_FFT_SIZE);
    return s_last_rms;
}

esp_err_t audio_processor_fft_forward(float *fft_io)
{
    if (fft_io == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = dsps_fft2r_fc32(fft_io, AUDIO_PROC_FFT_SIZE);
    if (err != ESP_OK) {
        return err;
    }
    return dsps_bit_rev_fc32(fft_io, AUDIO_PROC_FFT_SIZE);
}

void audio_processor_psd_from_fft(const float *fft_complex, float *psd_out)
{
    if (fft_complex == NULL || psd_out == NULL) {
        return;
    }
    for (int k = 0; k < AUDIO_PROC_PSD_BINS; k++) {
        const float re = fft_complex[2 * k];
        const float im = fft_complex[2 * k + 1];
        psd_out[k]     = re * re + im * im;
    }
}

esp_err_t audio_processor_compute_psd(float *fft_work, const int32_t *pcm, int n_samples)
{
    if (fft_work == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pcm != NULL) {
        audio_processor_prepare_fft_input(fft_work, pcm, n_samples);
    }
    esp_err_t err = audio_processor_fft_forward(fft_work);
    if (err != ESP_OK) {
        return err;
    }
    audio_processor_psd_from_fft(fft_work, s_psd);
    return ESP_OK;
}

const float *audio_processor_last_psd(void)
{
    return s_psd;
}

float audio_processor_last_rms(void)
{
    return s_last_rms;
}

#ifndef AUDIO_PROC_HARM_BIN_TOL
#define AUDIO_PROC_HARM_BIN_TOL 2
#endif

static int clamp_bin(int b, int n_bins)
{
    if (b < 0) {
        return 0;
    }
    if (b >= n_bins) {
        return n_bins - 1;
    }
    return b;
}

static float local_peak_max(const float *psd, int n_bins, int center, int half_width)
{
    int lo = clamp_bin(center - half_width, n_bins);
    int hi = clamp_bin(center + half_width, n_bins);
    float m = 0.f;
    for (int i = lo; i <= hi; i++) {
        if (psd[i] > m) {
            m = psd[i];
        }
    }
    return m;
}

static float local_window_mean(const float *psd, int n_bins, int center, int half_width)
{
    int lo = clamp_bin(center - half_width, n_bins);
    int hi = clamp_bin(center + half_width, n_bins);
    double sum = 0.0;
    int count = 0;
    for (int i = lo; i <= hi; i++) {
        sum += (double)psd[i];
        count++;
    }
    return (count > 0) ? (float)(sum / (double)count) : 1e-18f;
}

/* Estimate background noise floor while excluding the narrow harmonic peaks
 * (f0, 2*f0, 3*f0) so a strong drone signal does not inflate its own noise floor. */
static float estimate_clean_noise_floor(const float *psd, int n_bins_half,
                                        int f0_bin, int h2_bin, int h3_bin)
{
    const int start_bin = 5;
    double acc = 0.0;
    int count = 0;
    for (int i = start_bin; i < n_bins_half; i++) {
        if (abs(i - f0_bin) <= 2 || abs(i - h2_bin) <= 2 || abs(i - h3_bin) <= 2) {
            continue;
        }
        acc += (double)psd[i];
        count++;
    }
    if (count <= 0) {
        return audio_processor_noise_floor_estimate(psd, n_bins_half);
    }
    float nf = (float)(acc / (double)count);
    return (nf < 1e-18f) ? 1e-18f : nf;
}

bool analyze_harmonics(const float *psd, int n_bins_half, float f0_min_hz, float f0_max_hz,
                       HarmonicAnalysisResult *out)
{
    if (psd == NULL || out == NULL || n_bins_half < AUDIO_PROC_PSD_BINS) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    const SensitivityProfile *prof = &s_profiles[s_sensitivity_level - 1];

    int k0 = (int)ceilf(f0_min_hz / AUDIO_PROC_BIN_HZ);
    int k1 = (int)floorf(f0_max_hz / AUDIO_PROC_BIN_HZ);
    k0     = clamp_bin(k0, n_bins_half);
    k1     = clamp_bin(k1, n_bins_half);
    if (k1 <= k0) {
        return false;
    }

    /* Find candidate fundamental f0 with the strongest harmonic comb score
     * (f0 + 2*f0 + 3*f0), while also tracking the raw peak. */
    int peak_bin = k0;
    float best_comb = -1.f;
    for (int k = k0; k <= k1; k++) {
        float p1 = psd[k];
        float p2 = (k * 2 < n_bins_half) ? local_peak_max(psd, n_bins_half, k * 2, 1) : 0.f;
        float p3 = (k * 3 < n_bins_half) ? local_peak_max(psd, n_bins_half, k * 3, 1) : 0.f;
        /* Only consider k if p1 is a local maximum and dominates its harmonics */
        if (p1 >= p2 * 0.25f) {
            float comb = p1 + 1.5f * p2 + 1.5f * p3;
            if (comb > best_comb) {
                best_comb = comb;
                peak_bin  = k;
            }
        }
    }

    const float peak = local_peak_max(psd, n_bins_half, peak_bin, 1);
    const int h2c = peak_bin * 2;
    const int h3c = peak_bin * 3;

    const float nf = estimate_clean_noise_floor(psd, n_bins_half, peak_bin, h2c, h3c);
    out->noise_floor     = nf;
    const float snr      = peak / nf;
    out->fundamental_bin = peak_bin;
    out->fundamental_hz  = (float)peak_bin * AUDIO_PROC_BIN_HZ;
    out->fundamental_pwr = peak;
    out->snr             = snr;

    if (snr < prof->min_peak_snr || h2c >= n_bins_half || h3c >= n_bins_half) {
        out->confidence = 0.f;
        return false;
    }

    out->harmonic2_pwr = local_peak_max(psd, n_bins_half, h2c, AUDIO_PROC_HARM_BIN_TOL);
    out->harmonic3_pwr = local_peak_max(psd, n_bins_half, h3c, AUDIO_PROC_HARM_BIN_TOL);

    const float denom = peak + 1e-18f;
    out->h2_ratio     = out->harmonic2_pwr / denom;
    out->h3_ratio     = out->harmonic3_pwr / denom;

    /* Check harmonic SNR above background noise floor */
    const float h2_snr = out->harmonic2_pwr / nf;
    const float h3_snr = out->harmonic3_pwr / nf;

    /* Anti-false-alarm check: compare harmonic peaks against the inter-harmonic
     * valleys at 1.5*f0 and 2.5*f0. Broadband noise (speech, claps, wind, fans)
     * fills the valleys between harmonics, whereas a drone propeller has sharp
     * peaks at f0, 2*f0, 3*f0 and quiet valleys in between. */
    const int v1_bin = peak_bin + (peak_bin / 2);
    const int v2_bin = h2c + (peak_bin / 2);
    const float valley1 = local_window_mean(psd, n_bins_half, v1_bin, 1) + 1e-18f;
    const float valley2 = local_window_mean(psd, n_bins_half, v2_bin, 1) + 1e-18f;
    const bool valley_ok = (peak > valley1 * prof->min_valley_ratio) &&
                           (out->harmonic2_pwr > valley2 * (prof->min_valley_ratio * 0.7f));

    const bool h2_ok = (out->h2_ratio >= prof->min_h2_ratio) && (h2_snr >= prof->min_harm_snr);
    const bool h3_ok = (out->h3_ratio >= prof->min_h3_ratio) && (h3_snr >= prof->min_harm_snr * 0.8f);

    /* Normalized harmonic strength (1.0 at threshold, up to 3.0 for strong harmonics) */
    const float h_score = sqrtf((out->h2_ratio / prof->min_h2_ratio) *
                                (out->h3_ratio / prof->min_h3_ratio));

    if (!h2_ok || !h3_ok || !valley_ok) {
        out->confidence = fminf(0.22f, (snr / (prof->snr_norm_scale * 4.f)) * fminf(1.f, h_score * 0.3f));
        return false;
    }

    /* When all 3 checks pass (fundamental SNR, 2nd & 3rd harmonic SNR/ratio, and valley prominence),
     * compute confidence in [0.35 .. 1.0] so sustained harmonic tones trigger cleanly. */
    float raw_conf = (snr / prof->snr_norm_scale) * fminf(2.0f, 0.35f + 0.25f * h_score);
    out->confidence = fminf(1.0f, fmaxf(0.35f, raw_conf));
    return true;
}
