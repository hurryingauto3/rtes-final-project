#include <mbed.h>
#include "arm_math.h"

#include "ingest.hpp"

int main() {
  static BufferedSerial pc(USBTX, USBRX, 115200);
  if (!init_imu()) {
    printf("IMU not found; aborting!");
    while(1) { ThisThread::sleep_for(1s); }
  }

  Thread acq_thread;
  acq_thread.start(acquisition_task);

  // Holds an array of frequencies per axis [0, 26/128, ... , 26]Hz
  float accelerometer_frequencies[3][129], gyroscope_frequencies[3][129];

  ingest_batch_mutex.lock();
  while(1) {
    ingest_batch_condition.wait(); // Wait for a new batch of IMU data
    IMUBatch *imu_data = get_batch();
    // Respond to the batch of data

    for (int axis = 0; axis < 3; axis++) {
      arm_cmplx_mag_f32(
        imu_data->accelerometer[axis],
        accelerometer_frequencies[axis],
        256
      );
      arm_cmplx_mag_f32(
        imu_data->gyroscope[axis],
        gyroscope_frequencies[axis],
        256
      );
    }
  }
}