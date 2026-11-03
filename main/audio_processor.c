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
 * and digital mic gain (default 2.0x = +6 dB). */
static int   s_sensitivity_level = 3;
static float s_mic_gain          = 2.0f;
static bool  s_debug_log         = false;

typedef struct {
    float min_global_snr;   /* Minimum fundamental SNR above global noise floor */
    float min_f0_local_snr; /* Minimum f0 peak prominence above immediate neighboring bins */
    float min_h2_local_snr; /* Minimum 2*f0 peak prominence above its local neighboring bins */
    float min_h3_local_snr; /* Minimum 3*f0 peak prominence above its local neighboring bins */
    float min_h2_ratio;     /* Minimum P(2*f0)/P(f0) */
    float min_h3_ratio;     /* Minimum P(3*f0)/P(f0) */
    float snr_norm_scale;   /* Confidence normalization scale */
} SensitivityProfile;

/* Profiles 1..5:
 * Uses local narrowband SNR (peak vs bins [c-7..c-3, c+3..c+7]) so 1/f room noise
 * and broadband noise NEVER trigger false alarms, while true tonal harmonics do.
 */
static const SensitivityProfile s_profiles[5] = {
    /* 1: Strict (original-like) */ { 8.0f, 14.0f, 8.0f, 6.0f, 0.050f, 0.025f, 22.0f },
    /* 2: Conservative */           { 6.0f, 10.0f, 6.0f, 4.5f, 0.025f, 0.010f, 16.0f },
    /* 3: Balanced (default) */     { 4.5f,  7.5f, 4.5f, 3.5f, 0.012f, 0.005f, 12.0f },
    /* 4: High */                   { 3.2f,  5.5f, 3.8f, 3.0f, 0.005f, 0.002f,  8.0f },
    /* 5: Max */                    { 2.2f,  3.8f, 2.5f, 2.0f, 0.002f, 0.0005f, 5.5f },
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

void audio_processor_set_debug_log(bool enable)
{
    s_debug_log = enable;
    ESP_LOGI(TAG, "Audio 1Hz debug log %s", s_debug_log ? "ENABLED" : "DISABLED");
}

bool audio_processor_get_debug_log(void)
{
    return s_debug_log;
}

/* --- Default weak noise floor: mean linear power above 90 Hz (bin 6) --- */
__attribute__((weak)) float audio_processor_noise_floor_estimate(const float *psd, int n_bins_half)
{
    if (n_bins_half < 8 || psd == NULL) {
        return 1e-18f;
    }
    const int start_bin = 6;
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

static float s_channel_trim[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
static float s_noise_floor_baseline = 0.0003f;

void audio_processor_set_channel_trim(int ch, float trim)
{
    if (ch >= 0 && ch < 4) {
        if (trim < 0.10f) trim = 0.10f;
        if (trim > 10.0f) trim = 10.0f;
        s_channel_trim[ch] = trim;
    }
}

float audio_processor_get_channel_trim(int ch)
{
    if (ch >= 0 && ch < 4) {
        return s_channel_trim[ch];
    }
    return 1.0f;
}

void audio_processor_set_noise_floor_baseline(float rms)
{
    if (rms > 0.0f) {
        s_noise_floor_baseline = rms;
    }
}

float audio_processor_get_noise_floor_baseline(void)
{
    return s_noise_floor_baseline;
}

// cppcheck-suppress unusedFunction
const float *audio_processor_window(void)
{
    return s_window;
}

float audio_processor_prepare_fft_input_ch(float *fft_io, const int32_t *pcm, int n_samples, int ch)
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

    /* Step 2: Subtract DC, apply digital pre-gain and channel calibration trim, and apply Hanning window */
    double sum_sq = 0.0;
    const float trim = (ch >= 0 && ch < 4) ? s_channel_trim[ch] : 1.0f;
    const float gain = s_mic_gain * trim;

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

float audio_processor_prepare_fft_input(float *fft_io, const int32_t *pcm, int n_samples)
{
    return audio_processor_prepare_fft_input_ch(fft_io, pcm, n_samples, 0);
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

esp_err_t audio_processor_compute_psd_ch(float *fft_work, const int32_t *pcm, int n_samples, int ch)
{
    if (fft_work == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pcm != NULL) {
        audio_processor_prepare_fft_input_ch(fft_work, pcm, n_samples, ch);
    }
    esp_err_t err = audio_processor_fft_forward(fft_work);
    if (err != ESP_OK) {
        return err;
    }
    audio_processor_psd_from_fft(fft_work, s_psd);
    return ESP_OK;
}

esp_err_t audio_processor_compute_psd(float *fft_work, const int32_t *pcm, int n_samples)
{
    return audio_processor_compute_psd_ch(fft_work, pcm, n_samples, 0);
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

/* Compute local background noise in the sidebands [center - outer .. center - inner - 1]
 * and [center + inner + 1 .. center + outer].
 * Because 1/f room noise and broadband noise have high energy in these adjacent bins,
 * comparing a peak against its own local sidebands completely eliminates false alarms
 * in silence or broadband noise while remaining sensitive to true narrowband harmonics. */
static float local_sideband_noise(const float *psd, int n_bins, int center, int inner, int outer)
{
    double sum = 0.0;
    int count = 0;
    for (int d = inner + 1; d <= outer; d++) {
        int b_lo = center - d;
        int b_hi = center + d;
        if (b_lo >= 4) {
            sum += (double)psd[b_lo];
            count++;
        }
        if (b_hi < n_bins) {
            sum += (double)psd[b_hi];
            count++;
        }
    }
    if (count == 0) {
        return 1e-18f;
    }
    float avg = (float)(sum / (double)count);
    return (avg < 1e-18f) ? 1e-18f : avg;
}

bool analyze_harmonics(const float *psd, int n_bins_half, float f0_min_hz, float f0_max_hz,
                       HarmonicAnalysisResult *out)
{
    if (psd == NULL || out == NULL || n_bins_half < AUDIO_PROC_PSD_BINS) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    const SensitivityProfile *prof = &s_profiles[s_sensitivity_level - 1];
    const float nf = audio_processor_noise_floor_estimate(psd, n_bins_half);
    out->noise_floor = nf;

    int k0 = (int)ceilf(f0_min_hz / AUDIO_PROC_BIN_HZ);
    int k1 = (int)floorf(f0_max_hz / AUDIO_PROC_BIN_HZ);
    k0     = clamp_bin(k0, n_bins_half);
    k1     = clamp_bin(k1, n_bins_half);
    if (k1 <= k0) {
        return false;
    }

    /* Find the strongest fundamental peak in [f0_min_hz .. f0_max_hz] */
    int peak_bin = k0;
    float peak   = psd[k0];
    for (int k = k0 + 1; k <= k1; k++) {
        if (psd[k] > peak) {
            peak     = psd[k];
            peak_bin = k;
        }
    }

    const int h2c = peak_bin * 2;
    const int h3c = peak_bin * 3;

    /* Measure local narrowband SNR at f0, 2*f0, and 3*f0 against their immediate surrounding bins */
    const float f0_bg = local_sideband_noise(psd, n_bins_half, peak_bin, 2, 8);
    const float f0_local_snr = peak / f0_bg;
    const float global_snr   = peak / nf;

    out->fundamental_bin = peak_bin;
    out->fundamental_hz  = (float)peak_bin * AUDIO_PROC_BIN_HZ;
    out->fundamental_pwr = peak;
    out->snr             = f0_local_snr; /* Report local narrowband SNR */

    if (global_snr < prof->min_global_snr ||
        f0_local_snr < prof->min_f0_local_snr ||
        h2c >= n_bins_half || h3c >= n_bins_half) {
        out->confidence = 0.f;
        return false;
    }

    out->harmonic2_pwr = local_peak_max(psd, n_bins_half, h2c, AUDIO_PROC_HARM_BIN_TOL);
    out->harmonic3_pwr = local_peak_max(psd, n_bins_half, h3c, AUDIO_PROC_HARM_BIN_TOL);

    const float denom = peak + 1e-18f;
    out->h2_ratio     = out->harmonic2_pwr / denom;
    out->h3_ratio     = out->harmonic3_pwr / denom;

    const float h2_bg = local_sideband_noise(psd, n_bins_half, h2c, 2, 7);
    const float h3_bg = local_sideband_noise(psd, n_bins_half, h3c, 2, 7);
    const float h2_local_snr = out->harmonic2_pwr / h2_bg;
    const float h3_local_snr = out->harmonic3_pwr / h3_bg;

    /* In long-range detection (>20m), high frequencies attenuate rapidly in air.
     * At sensitivity 5, allow detection if either h2 is confirmed or h3 is confirmed.
     * At sensitivity 4, require h2 or strong h3.
     * At sensitivity 1..3, strictly require both h2 and h3. */
    const bool h2_ok = (out->h2_ratio >= prof->min_h2_ratio) && (h2_local_snr >= prof->min_h2_local_snr);
    const bool h3_ok = (out->h3_ratio >= prof->min_h3_ratio) && (h3_local_snr >= prof->min_h3_local_snr);
    const bool harm_ok = (s_sensitivity_level >= 5) ? (h2_ok || h3_ok) :
                         (s_sensitivity_level == 4) ? (h2_ok || (h3_ok && f0_local_snr >= 5.5f)) :
                         (h2_ok && h3_ok);

    if (!harm_ok) {
        out->confidence = fminf(0.15f, (f0_local_snr / (prof->snr_norm_scale * 4.f)));
        return false;
    }

    float h2_factor = fmaxf(1.0f, h2_local_snr / prof->min_h2_local_snr);
    float h3_factor = fmaxf(1.0f, h3_local_snr / prof->min_h3_local_snr);
    const float harm_prominence = sqrtf(h2_factor * h3_factor);
    float raw_conf = (f0_local_snr / prof->snr_norm_scale) * fminf(1.8f, 0.35f * harm_prominence);
    out->confidence = fminf(1.0f, fmaxf(0.32f, raw_conf));
    return true;
}
