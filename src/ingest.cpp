#include "ingest.hpp"

// Address info for the IMU chip
#define LSM6DSL_INT1_PIN    PD_11
#define LSM6DSL_ADDR        (0x6A << 1)

I2C i2c(PB_11, PB_10);

//MARK: Utility functions

// Read a single-byte register
bool read_reg(uint8_t reg, uint8_t &value) {
    char r = (char)reg;
    // Request the register, then read the response
    if (i2c.write(LSM6DSL_ADDR, &r, 1, true) != 0) return false;
    if (i2c.read(LSM6DSL_ADDR, &r, 1) != 0) return false;

    value = (uint8_t)r;
    return true;
}

// Write a single-byte register
bool write_reg(uint8_t reg, uint8_t val) {
    char buf[2] = { (char)reg, (char)val };
    return i2c.write(LSM6DSL_ADDR, buf, 2) == 0;
}

// Read a 16-bit signed integer from two consecutive registers
bool read_int16(uint8_t reg_low, int16_t &val) {
    uint8_t lo, hi;
    if (!read_reg(reg_low + 0, lo)) return false;
    if (!read_reg(reg_low + 1, hi)) return false;

    val = (int16_t)((hi << 8) | lo);
    return true;
}

// MARK: Runtime

#define OUTX_L_G   0x22 // Gyroscope X-axis low byte start address
#define OUTX_L_XL  0x28 // Accelerometer X-axis low byte start address

EventFlags imu_events;
#define EVT_FRAME_READY (1UL << 0)


Mutex ingest_batch_mutex;
// Use this in the main function to wait for batches of data.
// Waiting requires you to have locked the ingest_batch_mutex!
ConditionVariable ingest_batch_condition(ingest_batch_mutex);

IMUBatch flip_buffer[2];
bool flop;
uint8_t i_time;

#define I16_MAX 32767
#define ACCEL_SCALE (2.f / I16_MAX)
#define GYRO_SCALE 
void acquisition_task() {
    int16_t acc[3], gyro[3];
    while (1) {
        imu_events.wait_any(EVT_FRAME_READY);
        for (int axis = 0; axis < 3; axis++) {
            read_int16(OUTX_L_XL + 2*axis, acc[axis]);
            read_int16(OUTX_L_G  + 2*axis, gyro[axis]);
        }

        for (int axis = 0; axis < 3; axis++) {
            flip_buffer[flop].accelerometer[axis][i_time] = acc[axis] * (  2.f / I16_MAX);
            flip_buffer[flop].gyroscope[axis][i_time]     = acc[axis] * (250.f / I16_MAX);
        }

        if (i_time < BATCH_SIZE_FILLED) {
            i_time += 1;
        } else {
            // Ensure the rest of the flip buffer is clear
            for (int axis = 0; axis < 3; axis++) {
                memset(&flip_buffer[flop].accelerometer[axis][BATCH_SIZE_FILLED], 0, (BATCH_SIZE - BATCH_SIZE) * sizeof(float));
                memset(&flip_buffer[flop].gyroscope[axis][BATCH_SIZE_FILLED], 0, (BATCH_SIZE - BATCH_SIZE) * sizeof(float));
            }
            i_time = 0;
            flop = !flop;
            // Start running all tasks that consume the data.
            // We're using a mutex so that we won't overwrite stuff that's in progress
            // and will pause measuring if necessary to prevent it. We can only safely pause here, between batches,
            // because we are only calculating FFT-related stuff within a single batch.
            ingest_batch_mutex.lock();
            ingest_batch_condition.notify_all();
            ingest_batch_mutex.unlock();
        }
    }
}

IMUBatch* get_batch() {
    return &(flip_buffer[flop]);
}

InterruptIn int1(LSM6DSL_INT1_PIN, PullDown);

void data_ready_isr() { imu_events.set(EVT_FRAME_READY); }

//MARK: Setup

#define WHO_AM_I  0x0F // Device identification register

#define DRDY_PULSE_CFG  0x0B // Data-ready pulse configuration
#define INT1_CTRL       0x0D // INT1 pin routing control
#define CTRL1_XL        0x10 // Accelerometer control register
#define CTRL2_G         0x11 // Gyroscope control register
#define CTRL3_C         0x12 // Common control register

#define STATUS_REG 0x1E // Status register (data ready flags)

bool init_imu() {
    i2c.frequency(400000);

    // Verify that the sensor is present
    {
        uint8_t who;
        if (!read_reg(WHO_AM_I, who) || who != 0x6A) return false;
    }
    // polling_rate / 4 = 13Hz
    
    write_reg(CTRL3_C,   0x44); // Block updates, auto-increment address
    write_reg(CTRL2_G,   0x32); // Gyroscope:     52 Hz, ±250 dps, low pass: [polling_rate / 4]
    write_reg(CTRL1_XL,  0x32); // Accelerometer: 52 Hz, ±2 g, low pass: [polling_rate / 4]
    write_reg(INT1_CTRL, 0x03); // Route data-ready signal to INT1 pin
    write_reg(DRDY_PULSE_CFG, 0x80); // Enable pulsed data-ready mode (50μs pulses)

    // Wait for sensor to stabilize
    ThisThread::sleep_for(100ms);
    uint8_t dummy;
    read_reg(STATUS_REG, dummy);
    // Clear old data by reading all output registers
    int16_t temp;
    for (int i = 0; i < 6; i++) {
        read_int16(OUTX_L_XL + 2*i, temp);
    }

    int1.rise(&data_ready_isr);

    return true;
}