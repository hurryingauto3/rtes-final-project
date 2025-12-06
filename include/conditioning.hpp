#pragma once

//! Utility functions for processing digital data
//! Some functions are defined in the header so the compiler has the option of inlining them.

#include "globals.hpp"
#include "arm_math.h"

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
static void lowpass(float *data, FilterHistory2 *history, int n, bool invert_t, float *out) {
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
static void cross(const float a[3], const float b[3], float dest[3]) {
    dest[0] = a[1] * b[2] - a[2] * b[1];
    dest[1] = a[2] * b[0] - a[0] * b[2];
    dest[2] = a[0] * b[1] - a[1] * b[0];
}

//Parkinson's Disease Detection

/** Calculate tremor intensity in the 3-5 Hz frequency range from accelerometer data.
 * Sums frequency magnitudes across all 3 axes in the tremor band and normalizes.
 * @param accel_freq_mags Array of 3 frequency magnitude arrays (one per axis)
 * @return Tremor intensity value (0.0 = no tremor, higher values = more intense)
 */
static float detect_tremor(float accel_freq_mags[3][BATCH_SIZE / 2 + 1]) {
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
    int num_bins = (bin_5hz - bin_3hz + 1) * 3;
    float intensity = tremor_power / num_bins;
    
    return intensity;
}

/** Calculate dyskinesia intensity in the 5-7 Hz frequency range from accelerometer data.
 * Dyskinesia manifests as dance-like rhythmic movements in this frequency band.
 * @param accel_freq_mags Array of 3 frequency magnitude arrays (one per axis)
 * @return Dyskinesia intensity value (0.0 = none, higher values = more intense)
 */
static float detect_dyskinesia(float accel_freq_mags[3][BATCH_SIZE / 2 + 1]) {
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
    int num_bins = (bin_7hz - bin_5hz + 1) * 3;
    float intensity = dyskinesia_power / num_bins;
    
    return intensity;
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
static float detect_freezing(const float accel_time[3][BATCH_SIZE], float accel_freq_mags[3][BATCH_SIZE / 2 + 1]) {
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
    const float LOW_ACTIVITY_THRESHOLD = 0.05f;
    int low_activity_count = 0;
    const int N = BATCH_SIZE_FILLED;
    
    for (int t = 0; t < N; ++t) {
        float ax = accel_time[0][t];
        float ay = accel_time[1][t];
        float az = accel_time[2][t];
        float mag = sqrtf(ax * ax + ay * ay + az * az);
        
        if (mag < LOW_ACTIVITY_THRESHOLD) {
            low_activity_count += 1;
        }
    }
    float stillness_ratio = (float)low_activity_count / (float)N;
    
    // === Step 3: State machine ===
    const float WALKING_THRESHOLD = 0.5f;  // Tune based on your data
    const float STILLNESS_THRESHOLD = 0.7f; // 70% of samples must be still
    const int MIN_WALKING_BATCHES = 2;     // Must walk for at least 2 batches (6 seconds)
    const int FREEZE_DECAY_BATCHES = 3;    // Alert decays after 3 batches without movement
    
    bool is_walking = (walking_intensity > WALKING_THRESHOLD) && (stillness_ratio < 0.5f);
    bool is_still = (stillness_ratio > STILLNESS_THRESHOLD);
    
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
    
    return intensity;
}
