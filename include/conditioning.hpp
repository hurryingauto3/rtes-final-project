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
// Freezing-of-Gait detection (time-domain)
/**
 * Estimate FOG intensity from time-domain accelerometer data.
 *
 * The accel_time input should already be in the global frame, low-pass filtered,
 * and gravity-compensated (matching how IMUBatch.accelerometer is set up).
 *
 * Basic idea: normal walking shows larger, varying dynamic acceleration. During
 * a freeze, acceleration stays near zero for most of the batch. We count how many
 * samples fall below a small threshold and turn that into a 0-1 intensity score.
 *
 * @param accel_time 3xBATCH_SIZE array of filtered accel samples (gravity removed)
 * @return FOG intensity [0.0, 1.0] where higher means longer low-motion period
 */
static float detect_freezing(const float accel_time[3][BATCH_SIZE]) {
    // Threshold for "very low" dynamic acceleration (in g units).
    // This can be tuned empirically using recorded data.
    const float LOW_ACTIVITY_THRESHOLD = 0.05f;

    int low_activity_count = 0;

    // Use only the filled portion of the batch to avoid counting zero-padded
    // samples as artificial "freezes".
    const int N = BATCH_SIZE_FILLED;

    for (int t = 0; t < N; ++t) {
        float ax = accel_time[0][t];
        float ay = accel_time[1][t];
        float az = accel_time[2][t];

        // Since gravity has already been subtracted (see ingestion),
        // the magnitude itself represents dynamic motion.
        float mag = sqrtf(ax * ax + ay * ay + az * az);

        if (mag < LOW_ACTIVITY_THRESHOLD) {
            low_activity_count += 1;
        }
    }

    // Convert to a fraction in [0, 1].
    float intensity = 0.0f;
    if (N > 0) {
        intensity = (float)low_activity_count / (float)N;
    }

    // Clamp to the valid range.
    if (intensity < 0.0f) intensity = 0.0f;
    if (intensity > 1.0f) intensity = 1.0f;

    return intensity;
}
