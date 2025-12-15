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
 * We use a simple state machine across batches to track walking -> freeze transitions.
 * 
 * @param accel_time 3xBATCH_SIZE array of filtered accel samples (gravity removed)
 * @param accel_freq_mags Frequency domain representation for step detection
 * @return FOG intensity [0.0, 1.0] where higher means more confident freeze after walking
 */
inline static float detect_freezing(const float accel_time[3][BATCH_SIZE], float accel_freq_mags[3][BATCH_SIZE / 2 + 1]) {
    static enum { IDLE, WALKING, FROZEN } fog_state = IDLE;
    static int walking_batch_count = 0;
    static int frozen_batch_count = 0;
    
    // === Step 1: Detect if currently walking ===
    // Walking typically shows rhythmic motion in 1-3 Hz (cadence ~60-180 steps/min)
    int bin_1hz = (int)(1.0f / FREQUENCY_BIN_SIZE);
    int bin_3hz = (int)(3.0f / FREQUENCY_BIN_SIZE);
    
    float walking_power = 0.0f;
    for (int axis = 0; axis < 3; axis++) {
        for (int bin = bin_1hz; bin <= bin_3hz; bin++) {
            walking_power += accel_freq_mags[axis][bin];
        }
    }
    int num_walking_bins = (bin_3hz - bin_1hz + 1) * 3;
    float walking_intensity = walking_power / num_walking_bins;
    
    // === Step 2: Detect low motion (potential freeze) ===
    // Calculate variance to detect stillness (low variance = not moving much)
    const int N = BATCH_SIZE_FILLED;
    float mean[3] = {0.0f, 0.0f, 0.0f};
    
    // Calculate mean for each axis
    for (int t = 0; t < N; ++t) {
        mean[0] += accel_time[0][t];
        mean[1] += accel_time[1][t];
        mean[2] += accel_time[2][t];
    }
    mean[0] /= N;
    mean[1] /= N;
    mean[2] /= N;
    
    // Calculate variance (measure of motion)
    float variance = 0.0f;
    for (int t = 0; t < N; ++t) {
        float dx = accel_time[0][t] - mean[0];
        float dy = accel_time[1][t] - mean[1];
        float dz = accel_time[2][t] - mean[2];
        variance += (dx*dx + dy*dy + dz*dz);
    }
    variance /= N;
    
    // Low variance = stillness. Typical walking has variance > 0.1
    const float STILLNESS_VARIANCE_THRESHOLD = 0.08f;
    float stillness_ratio = (variance < STILLNESS_VARIANCE_THRESHOLD) ? 1.0f : 0.0f;
    
    // === Step 3: State machine ===
    // More realistic thresholds for actual gait detection
    const float WALKING_THRESHOLD = 0.10f;  // Very sensitive to walking motion
    const int MIN_WALKING_BATCHES = 1;      // Reduced to 1 batch (3 seconds) for quicker detection
    const int FREEZE_DECAY_BATCHES = 5;     // Increased to 5 batches (~15 seconds) to keep signal visible
    
    bool is_walking = (walking_intensity > WALKING_THRESHOLD) && (variance > 0.03f);  // Require some variance to be walking
    bool is_still = (variance < 0.06f);  // Stillness when variance drops below this
    
    switch (fog_state) {
        case IDLE:
            if (is_walking) {
                fog_state = WALKING;
                walking_batch_count = 1;
                frozen_batch_count = 0;
            }
            break;
            
        case WALKING:
            if (is_walking) {
                walking_batch_count++;
                frozen_batch_count = 0;
            } else if (is_still && walking_batch_count >= MIN_WALKING_BATCHES) {
                // Transition to freeze only if we were walking long enough
                fog_state = FROZEN;
                frozen_batch_count = 1;
            } else if (!is_walking && !is_still) {
                // Ambiguous state, reset
                fog_state = IDLE;
                walking_batch_count = 0;
            }
            break;
            
        case FROZEN:
            if (is_still) {
                frozen_batch_count++;
            } else if (is_walking) {
                // Recovered from freeze, back to walking
                fog_state = WALKING;
                walking_batch_count = 1;
                frozen_batch_count = 0;
            } else {
                // Decay the freeze alert
                frozen_batch_count++;
                if (frozen_batch_count > FREEZE_DECAY_BATCHES) {
                    fog_state = IDLE;
                    frozen_batch_count = 0;
                    walking_batch_count = 0;
                }
            }
            break;
    }
    
    // === Step 4: Calculate intensity ===
    float intensity = 0.0f;
    if (fog_state == FROZEN && frozen_batch_count > 0) {
        // Ramp up intensity based on how long we've been frozen
        // Cap at 1.0 after FREEZE_DECAY_BATCHES
        intensity = (float)frozen_batch_count / (float)FREEZE_DECAY_BATCHES;
        if (intensity > 1.0f) intensity = 1.0f;
    }
    
    #ifdef DEBUG
    // Debug output to understand state transitions - print every batch for real-time visibility
    printf("FOG: state=%d, walk=%.2f, var=%.3f, int=%.3f\n",
           fog_state, walking_intensity, variance, intensity);
    #endif
    
    return intensity;
}
