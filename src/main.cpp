#include <mbed.h>
#include "arm_math.h"

#include "ingest.hpp"

void do_fft(float32_t data[BATCH_SIZE], float32_t out[BATCH_SIZE / 2 + 1]);

int main() {
  static BufferedSerial pc(USBTX, USBRX, 115200);
  if (!init_imu()) {
    printf("IMU not found; aborting!");
    while(1) { ThisThread::sleep_for(1s); }
  }

  Thread acq_thread;
  acq_thread.start(acquisition_task);

  // Holds an array of frequencies per axis [0, 26/128, ... , 26]Hz
  float accelerometer_frequency_magnitudes[3][BATCH_SIZE / 2 + 1], gyroscope_frequency_magnitudes[3][BATCH_SIZE / 2 + 1];

  ingest_batch_mutex.lock();
  while(1) {
    ingest_batch_condition.wait(); // Wait for a new batch of IMU data
    IMUBatch *imu_data = get_batch();
    // Respond to the batch of data

    for (int axis = 0; axis < 3; axis++) {
      do_fft(imu_data->accelerometer[axis], accelerometer_frequency_magnitudes[axis]);
      do_fft(imu_data->gyroscope[axis], gyroscope_frequency_magnitudes[axis]);
    }
  }
}

void do_fft(float32_t data[BATCH_SIZE], float32_t out[BATCH_SIZE / 2 + 1]) {
  static float32_t complex_fft_coefficients[BATCH_SIZE * 2];
  static arm_rfft_fast_instance_f32 fft_instance;
  static arm_status fft_status = arm_rfft_fast_init_f32(&fft_instance, BATCH_SIZE);

  arm_rfft_fast_f32(&fft_instance, data, complex_fft_coefficients, 0);
  arm_cmplx_mag_f32(
    complex_fft_coefficients,
    out,
    BATCH_SIZE / 2 + 1
  );
}