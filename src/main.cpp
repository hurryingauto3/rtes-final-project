#include <mbed.h>

#include "globals.hpp"
#include "ingest.hpp"
#include "conditioning.hpp"
#include "ble_handler.hpp"

// BLE Objects
static events::EventQueue ble_event_queue(16 * EVENTS_EVENT_SIZE);
static Thread ble_thread;
static ParkinsonBLE ble_handler(ble_event_queue);

int main() {
  static BufferedSerial pc(USBTX, USBRX, 115200);

  // Start BLE Thread
  ble_thread.start(callback(&ble_event_queue, &events::EventQueue::dispatch_forever));
  ble_handler.init();

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

    // TODO: Calculate these values based on your FFT analysis
    float tremor_intensity = detect_tremor(accelerometer_frequency_magnitudes);
    float dyskinesia_intensity = 0.0f;
    // Freezing-of-Gait detection using time-domain accelerometer data.
    float fog_intensity = detect_freezing(imu_data->accelerometer);


    // Update BLE characteristics
    ble_handler.updateTremor(tremor_intensity);
    ble_handler.updateDyskinesia(dyskinesia_intensity);
    ble_handler.updateFreezingGait(fog_intensity);

    #ifdef TELEPLOT
      // Print in Teleplot format (>name:value)
      printf(">tremor_intensity:%.3f\n>dyskinesia_intensity:%.3f\n>fog_intensity:%.3f\n",
        tremor_intensity, dyskinesia_intensity, fog_intensity
      );
      #endif
  }

  return 0;
}