#include <mbed.h>

#include "globals.hpp"
#include "ingest.hpp"

int main() {
  static BufferedSerial pc(USBTX, USBRX, 115200);

  #ifdef DEBUG
  if (!init_imu()) {
    printf("IMU not found; aborting!");
    while(1) { ThisThread::sleep_for(1s); }
  }
  #else
  init_imu();
  #endif

  Thread acq_thread;
  acq_thread.start(acquisition_task);

  // Holds an array of frequencies per axis [0, 26/128, ... , 26]Hz
  float accelerometer_frequency_magnitudes[3][BATCH_SIZE / 2 + 1], gyroscope_frequency_magnitudes[3][BATCH_SIZE / 2 + 1];
  init_fft();

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

  return 0;
}