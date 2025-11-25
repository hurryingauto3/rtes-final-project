#pragma once

#include <mbed.h>

#include "globals.hpp"

#define DEBUG_WARN_OVERFLOW

extern I2C i2c;

extern Mutex ingest_batch_mutex;
// Use this in the main function to wait for batches of data.
// Waiting requires you to have locked the ingest_batch_mutex!
extern ConditionVariable ingest_batch_condition;

/// @brief Attempt to set up the IMU
/// @return true if setup completed successfully
bool init_imu();

/// @brief Main loop to gather data from the IMU
void acquisition_task();

typedef struct {
    float accelerometer[3][BATCH_SIZE];
    float gyroscope[3][BATCH_SIZE];
} IMUBatch;

/** Retrieve the currently readable batch of IMU data. */
IMUBatch* get_batch();

/** Perform setup for the FFT */
void init_fft();

/** Run the FFT on some data to get an array of frequency magnitudes. */
void do_fft(float data[BATCH_SIZE], float frequency_magnitudes[BATCH_SIZE / 2 + 1]);