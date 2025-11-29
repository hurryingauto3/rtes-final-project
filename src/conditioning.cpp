#include "arm_math.h"
#include "conditioning.hpp"
#include "globals.hpp"

// MARK: FFT

arm_rfft_fast_instance_f32 fft_instance;
float32_t complex_fft_coefficients[BATCH_SIZE * 2];

void init_fft() {
    arm_rfft_fast_init_f32(&fft_instance, BATCH_SIZE);
}

void do_fft(float data[BATCH_SIZE], float frequency_magnitudes[BATCH_SIZE / 2 + 1]) {
  arm_rfft_fast_f32(&fft_instance, data, complex_fft_coefficients, 0);
  arm_cmplx_mag_f32(
    complex_fft_coefficients,
    frequency_magnitudes,
    BATCH_SIZE / 2 + 1
  );
}