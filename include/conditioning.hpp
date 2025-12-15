#pragma once

//! Utility functions for processing digital data
//! Some functions are defined in the header so the compiler has the option of inlining them.

#include "globals.hpp"
#include "arm_math.h"
#include <cstdio>

// How fast gravity moves toward the average 
#define GRAVITY_UPDATE_RATE 0.05
// Slows gravity updates when in motion
#define MOTION_SENSITIVITY 16

//MARK: FFT

/** Perform setup for the FFT */
void init_fft();

/** Run the FFT on some data to get an array of frequency magnitudes. */
void do_fft(float data[BATCH_SIZE], float frequency_magnitudes[BATCH_SIZE / 2 + 1]);

//MARK: Batch operations

// History data for 2nd order recursive filters
typedef struct {
    float x[2];
    float y[2];
} FilterHistory2;

/** Apply a low pass to a series of data points. Out may be the same as data.
 * @details a 2nd order Chebyshev-I low pass w/ 2db passband ripple, 7Hz cutoff
*/
inline static void lowpass(float *data, FilterHistory2 *history, int n, bool invert_t, float *out) {
    for (int t = 0; t < n; t++) {
        bool odd = ((t & 1) == 0) ^ invert_t;
        float y = 
            0.0866 * data[t] + 0.1733 * history->x[odd ? 1 : 0] + history->x[!odd ? 1 : 0] * 0.0866
                             + 1.0903 * history->y[odd ? 1 : 0] - history->y[!odd ? 1 : 0] * 0.5266;
        history->x[!odd ? 1 : 0] = data[t];
        history->y[!odd ? 1 : 0] = y;
        out[t] = y;
    }
}

/** Cross product creates a vector that is perpendicular to both a and b */
inline static void cross(const float a[3], const float b[3], float dest[3]) {
    dest[0] = a[1] * b[2] - a[2] * b[1];
    dest[1] = a[2] * b[0] - a[0] * b[2];
    dest[2] = a[0] * b[1] - a[1] * b[0];
}

inline static float calc_total_energy(float accel_freq_mags[3][BATCH_SIZE / 2 + 1]) {
    float total[3] = { 0.f, 0.f, 0.f };
    for (int axis = 0; axis < 3; axis++) {
        for (int bin = 0; bin < BATCH_SIZE / 2 + 1; bin++) {
            total[axis] += accel_freq_mags[axis][bin];
        }
    }
    // Equivalent to Euclidean length
    float ret;
    arm_sqrt_f32(total[0] * total[0] + total[1] * total[1] + total[2] * total[2], &ret);
    return ret;
}

//Parkinson's Disease Detection

/** Calculate tremor intensity in the 3-5 Hz frequency range from accelerometer data.
 * Sums frequency magnitudes across all 3 axes in the tremor band and normalizes.
 * @param accel_freq_mags Array of 3 frequency magnitude arrays (one per axis)
 * @return Tremor intensity value (0.0 = no tremor, higher values = more intense)
 */
inline static float detect_tremor(float accel_freq_mags[3][BATCH_SIZE / 2 + 1]) {
    // Frequency bin calculation: bin_size = POLL_RATE / BATCH_SIZE = 52/256 ≈ 0.203 Hz/bin
    // 3 Hz → bin ~15, 5 Hz → bin ~25
    int bin_3hz = (int)(3.0f / FREQUENCY_BIN_SIZE);
    int bin_5hz = (int)(5.0f / FREQUENCY_BIN_SIZE);
    
    float tremor_power = 0.0f;
    
    // Sum magnitude across frequency range and all 3 axes
    for (int axis = 0; axis < 3; axis++) {
        for (int bin = bin_3hz; bin <= bin_5hz; bin++) {
            tremor_power += accel_freq_mags[axis][bin];
        }
    }
    
    // Normalize by number of bins and axes for consistent intensity metric
    //int num_bins = (bin_5hz - bin_3hz + 1) * 3;
    //float intensity = tremor_power;
    
    return tremor_power;
}

/** Calculate dyskinesia intensity in the 5-7 Hz frequency range from accelerometer data.
 * Dyskinesia manifests as dance-like rhythmic movements in this frequency band.
 * @param accel_freq_mags Array of 3 frequency magnitude arrays (one per axis)
 * @return Dyskinesia intensity value (0.0 = none, higher values = more intense)
 */
inline static float detect_dyskinesia(float accel_freq_mags[3][BATCH_SIZE / 2 + 1]) {
    // Frequency bin calculation: bin_size = POLL_RATE / BATCH_SIZE = 52/256 ≈ 0.203 Hz/bin
    // 5 Hz → bin ~25, 7 Hz → bin ~34
    int bin_5hz = (int)(5.0f / FREQUENCY_BIN_SIZE);
    int bin_7hz = (int)(7.0f / FREQUENCY_BIN_SIZE);
    
    float dyskinesia_power = 0.0f;
    
    // Sum magnitude across frequency range and all 3 axes
    for (int axis = 0; axis < 3; axis++) {
        for (int bin = bin_5hz; bin <= bin_7hz; bin++) {
            dyskinesia_power += accel_freq_mags[axis][bin];
        }
    }
    
    // Normalize by number of bins and axes for consistent intensity metric
    // int num_bins = (bin_7hz - bin_5hz + 1) * 3;
    // float intensity = dyskinesia_power / num_bins;
    
    return dyskinesia_power;
}

// Freezing-of-Gait detection (time-domain + state tracking)
/**
 * Enhanced FOG detection that looks for the characteristic pattern:
 * 1. Walking detected (rhythmic movement in 1-3 Hz range, typically ~2 Hz for steps)
 * 2. Sudden cessation of movement (low dynamic acceleration)
 *
 * We use a state machine across batches to track walking -> freeze transitions.
 * Thresholds are calibrated at runtime from the device's own idle variance.
 *
 * @param accel_time 3xBATCH_SIZE array of filtered accel samples (gravity removed)
 * @param accel_freq_mags Frequency domain representation for step detection
 * @return FOG intensity [0.0, 1.0] where higher means more confident freeze after walking
 */
inline static float detect_freezing(const float accel_time[3][BATCH_SIZE], float accel_freq_mags[3][BATCH_SIZE / 2 + 1]) {
    // --- Persistent state across batches ---
    static enum { IDLE, WALKING, FROZEN } fog_state = IDLE;
    static int  walking_batch_count = 0;
    static int  frozen_batch_count  = 0;
    static int  still_batch_count   = 0;

    // Calibration of variance thresholds (idle noise floor)
    static bool  calibrated = false;
    static bool  calibration_just_completed = false;
    static float idle_var_mean = 0.0f;
    static float idle_var_m2   = 0.0f;
    static int   idle_n        = 0;
    static float STILLNESS_VARIANCE_THRESHOLD = 0.0f;
    static float WALKING_VARIANCE_THRESHOLD   = 0.0f;

    // --- Step 1: Compute walking intensity (1–3 Hz band) with safe bins ---
    const int   N_FFT  = BATCH_SIZE_FILLED;
    const float bin_hz = (float)POLL_RATE / (float)N_FFT; // Hz per FFT bin

    int bin_1hz = (int)lroundf(1.0f / bin_hz);
    int bin_3hz = (int)lroundf(3.0f / bin_hz);

    int max_bin = N_FFT / 2;
    if (bin_1hz < 0)       bin_1hz = 0;
    if (bin_3hz > max_bin) bin_3hz = max_bin;
    if (bin_1hz > bin_3hz) {
        bin_1hz = 0;
        bin_3hz = 0;
    }

    float walking_power = 0.0f;
    for (int axis = 0; axis < 3; axis++) {
        for (int bin = bin_1hz; bin <= bin_3hz; bin++) {
            walking_power += accel_freq_mags[axis][bin];
        }
    }

    int num_walking_bins = (bin_3hz - bin_1hz + 1) * 3;
    if (num_walking_bins <= 0) {
        num_walking_bins = 1; // avoid divide-by-zero
    }
    float walking_intensity = walking_power / (float)num_walking_bins;

    // --- Step 2: Time-domain variance as motion metric ---
    const int N = BATCH_SIZE_FILLED;
    float mean[3] = {0.0f, 0.0f, 0.0f};

    for (int t = 0; t < N; ++t) {
        mean[0] += accel_time[0][t];
        mean[1] += accel_time[1][t];
        mean[2] += accel_time[2][t];
    }
    mean[0] /= N;
    mean[1] /= N;
    mean[2] /= N;

    float variance = 0.0f;
    for (int t = 0; t < N; ++t) {
        float dx = accel_time[0][t] - mean[0];
        float dy = accel_time[1][t] - mean[1];
        float dz = accel_time[2][t] - mean[2];
        variance += (dx * dx + dy * dy + dz * dz);
    }
    variance /= N;

    // --- Step 3: Short idle calibration to learn noise floor ---
    if (!calibrated) {
        // Online variance estimation (Welford)
        idle_n++;
        float d = variance - idle_var_mean;
        idle_var_mean += d / (float)idle_n;
        idle_var_m2   += d * (variance - idle_var_mean);

        if (idle_n >= 20) { // ~20 initial batches assumed idle
            float idle_std = 0.0f;
            if (idle_n > 1 && idle_var_m2 > 0.0f) {
                idle_std = sqrtf(idle_var_m2 / (float)(idle_n - 1));
            }

            if (idle_std > 0.0f) {
                // Normal case: thresholds relative to noise floor
                STILLNESS_VARIANCE_THRESHOLD = idle_var_mean + 2.5f * idle_std;
                WALKING_VARIANCE_THRESHOLD   = idle_var_mean + 6.0f * idle_std;
            } else {
                // Fallback: calibration saw effectively zero variance (e.g. accel_time all zeros).
                // Use defaults tuned to your observed scale: idle var ~0.08 when resting,
                // vigorous motion >> 0.5.
                STILLNESS_VARIANCE_THRESHOLD = 0.10f;  // idle 0.08 < 0.10 ⇒ counts as still
                WALKING_VARIANCE_THRESHOLD   = 0.15f;  // requires clear motion to count as walking
            }
            calibrated = true;
            calibration_just_completed = true;

            #ifdef DEBUG
            printf("FOG calib: idle_mean=%.4f idle_std=%.4f stillThr=%.4f walkVarThr=%.4f\n",
                   idle_var_mean, idle_std,
                   STILLNESS_VARIANCE_THRESHOLD, WALKING_VARIANCE_THRESHOLD);
            #endif
        }

        // Until calibrated, stay idle with zero intensity
        return 0.0f;
    }

    // --- Step 4: Walking hysteresis + stillness using calibrated thresholds ---
    const float WALK_ON  = 0.11f;
    const float WALK_OFF = 0.08f;
    static bool walking_flag = false;

    if (!walking_flag) {
        walking_flag = (walking_intensity > WALK_ON);
    } else {
        walking_flag = (walking_intensity > WALK_OFF);
    }

    bool is_still   = (variance < STILLNESS_VARIANCE_THRESHOLD);
    bool is_walking = walking_flag && (variance > WALKING_VARIANCE_THRESHOLD);

    // --- Step 5: State machine with walking->stillness requirement ---
    const int MIN_WALKING_BATCHES  = 3;  // require several walking windows
    const int MIN_STILL_BATCHES    = 2;  // require stillness persistence
    const int FREEZE_DECAY_BATCHES = 5;  // how long FOG stays high

    switch (fog_state) {
        case IDLE:
            if (is_walking) {
                fog_state = WALKING;
                walking_batch_count = 1;
                still_batch_count   = 0;
                frozen_batch_count  = 0;
            }
            break;

        case WALKING:
            if (is_walking) {
                walking_batch_count++;
                still_batch_count = 0;
            } else if (is_still && walking_batch_count >= MIN_WALKING_BATCHES) {
                still_batch_count++;
                if (still_batch_count >= MIN_STILL_BATCHES) {
                    fog_state = FROZEN;
                    frozen_batch_count = 1;
                }
            } else {
                // Neither confidently walking nor still after good walking
                fog_state = IDLE;
                walking_batch_count = 0;
                still_batch_count   = 0;
                frozen_batch_count  = 0;
            }
            break;

        case FROZEN:
            if (is_still) {
                frozen_batch_count++;
                if (frozen_batch_count > FREEZE_DECAY_BATCHES) {
                    fog_state = IDLE;
                    walking_batch_count = 0;
                    still_batch_count   = 0;
                    frozen_batch_count  = 0;
                }
            } else if (is_walking) {
                // Recovered from freeze, back to walking
                fog_state = WALKING;
                walking_batch_count = 1;
                still_batch_count   = 0;
                frozen_batch_count  = 0;
            } else {
                // Ambiguous motion; decay the freeze alert
                frozen_batch_count++;
                if (frozen_batch_count > FREEZE_DECAY_BATCHES) {
                    fog_state = IDLE;
                    walking_batch_count = 0;
                    still_batch_count   = 0;
                    frozen_batch_count  = 0;
                }
            }
            break;
    }

    // --- Step 6: Calculate intensity ---
    float intensity = 0.0f;
    if (fog_state == FROZEN && frozen_batch_count > 0) {
        intensity = (float)frozen_batch_count / (float)FREEZE_DECAY_BATCHES;
        if (intensity > 1.0f) intensity = 1.0f;
    }

    #ifdef DEBUG
    printf("FOGdbg N=%d binHz=%.4f b1=%d b3=%d walkPow=%.4f walkInt=%.4f "
           "var=%.4f stillThr=%.4f walkVarThr=%.4f state=%d walkB=%d stillB=%d freezeB=%d int=%.3f\n",
           N_FFT, bin_hz, bin_1hz, bin_3hz,
           walking_power, walking_intensity,
           variance, STILLNESS_VARIANCE_THRESHOLD, WALKING_VARIANCE_THRESHOLD,
           fog_state, walking_batch_count, still_batch_count, frozen_batch_count, intensity);
    #endif

    // Send a one-shot special calibration marker via FOG:
    // Freezing of Gait: -0.10 will be interpreted by the Python UI as
    // "calibration complete", then normal intensities resume.
    if (calibration_just_completed) {
        calibration_just_completed = false;
        return -0.10f;
    }

    return intensity;
}
