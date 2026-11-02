/*
 * doa_processor.h — Multi-microphone Direction of Arrival (DoA) estimator
 *
 * Uses frequency-domain SNR-weighted SRP-PHAT (Steered Response Power with
 * Phase Transform) over both the detected drone harmonic bins and high-SNR
 * broadband bins from the 1024-point ESP-DSP FFT.
 *
 * Supports:
 *   - 4-mic planar cross ("kruisverband"):
 *       Mic 0 (DIN=GPIO6, SEL=GND): Noord / Voor   (0°)
 *       Mic 1 (DIN=GPIO6, SEL=3V3): Zuid  / Achter (180°)
 *       Mic 2 (DIN=GPIO7, SEL=GND): Oost  / Rechts (90°)
 *       Mic 3 (DIN=GPIO7, SEL=3V3): West  / Links  (270°)
 *     Outputs 360° Azimuth [0..359°] AND 3D Elevation [0..90°] in degrees.
 *   - 2-mic linear array fallback (when only Bus 0 has mics connected):
 *     Outputs 1D Azimuth [0..180°].
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
    bool    valid;                  /* true if >= 2 microphones had sufficient signal energy */
    int     active_mics;            /* number of active microphone channels (2 or 4)         */
    float   azimuth_deg;            /* azimuth in degrees: [0..360) for 4-mic, [0..180] 2-mic */
    float   elevation_deg;          /* elevation in degrees: [0..90] for 4-mic, -1 for 2-mic */
    float   confidence;             /* SRP-PHAT peak coherence [0.0 .. 1.0]                  */
    float   rms_left;               /* normalized RMS of Ch0 (Noord / Left, SEL=GND)         */
    float   rms_right;              /* normalized RMS of Ch1 (Zuid / Right, SEL=3V3)         */
    float   rms_ch[DOA_MAX_MICS];   /* normalized RMS of all 4 channels [N, Z, O, W]         */
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

const char *doa_angle_to_cardinal(int16_t deg, int active_mics);

void doa_simulate_multichannel_pcm(int32_t *pcm_ch[],
                                   int num_mics,
                                   int n_samples,
                                   float mic_spacing_mm,
                                   float f0_hz,
                                   float azimuth_deg);

#ifdef __cplusplus
}
#endif
