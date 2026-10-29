/*
 * doa_processor.h — Multi-microphone Direction of Arrival (DoA) estimator
 *
 * Uses frequency-domain SRP-PHAT (Steered Response Power with Phase Transform)
 * over the detected drone harmonic bins from the existing 1024-point ESP-DSP FFT.
 *
 * 2-mic linear array (1D front hemisphere):
 *   0° = Left (LINKS), 90° = Front/Center (MIDDEN), 180° = Right (RECHTS)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "audio_processor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DOA_MAX_MICS        4
#define DOA_INVALID_DEG     (-1)

typedef struct {
    bool    valid;          /* true if >= 2 microphones had sufficient signal energy */
    int     active_mics;    /* number of active microphone channels (2 or 4)         */
    float   azimuth_deg;    /* estimated direction in degrees [0.0 .. 180.0]         */
    float   confidence;     /* SRP-PHAT peak coherence [0.0 .. 1.0]                  */
    float   rms_left;       /* normalized RMS of Left mic (SEL=GND)                  */
    float   rms_right;      /* normalized RMS of Right mic (SEL=3V3)                 */
} DoaResult;

esp_err_t doa_processor_init(void);
void      doa_processor_deinit(void);
void      doa_processor_set_spacing_mm(float spacing_mm);
float     doa_processor_get_spacing_mm(void);

esp_err_t doa_processor_estimate(const int32_t * const pcm_ch[],
                                 int num_mics,
                                 float mic_spacing_mm,
                                 int f0_bin,
                                 DoaResult *out);

const char *doa_angle_to_cardinal(int16_t deg);

void doa_simulate_multichannel_pcm(int32_t *pcm_ch[],
                                   int num_mics,
                                   int n_samples,
                                   float mic_spacing_mm,
                                   float f0_hz,
                                   float azimuth_deg);

#ifdef __cplusplus
}
#endif
