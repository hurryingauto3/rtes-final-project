#pragma once

#include <mbed.h>

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

#define POLL_RATE 52
#define BATCH_SIZE_FILLED (3 * POLL_RATE) // 156
#define BATCH_SIZE 256 // Next highest power of 2

typedef struct {
    float accelerometer[3][BATCH_SIZE];
    float gyroscope[3][BATCH_SIZE];
} IMUBatch;

/** Retrieve the currently readable batch of IMU data. */
IMUBatch* get_batch();
