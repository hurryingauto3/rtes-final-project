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
