#include "ingest.hpp"

#include "arm_math.h"

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

// Chebyshev-I low pass
// n=2, 2db pass-band ripple, 7hz cutoff
struct LowPass {
    float32_t x[2];
    float32_t y[2];
};

LowPass accel_hist[3];
LowPass gyro_hist[3];

float32_t low_pass(LowPass *hist, float32_t x, bool even_t) {
    float y = (
        x * 0.0866 + hist->x[even_t ? 1 : 0] * 0.1733 + hist->x[even_t ? 0 : 1] * 0.0866
                   + hist->y[even_t ? 1 : 0] * 1.0903 - hist->y[even_t ? 0 : 1] * 0.5266
    );
    hist->x[even_t ? 0 : 1] = x;
    hist->y[even_t ? 0 : 1] = y;
    return y;
}

#define cross(a, b) { \
    a[1] * b[2] - a[2] * b[1], \
    a[2] * b[0] - a[0] * b[2], \
    a[0] * b[1] - a[1] * b[0]  \
}

#define I16_MAX 32767
#define ACCEL_SCALE (2.f / I16_MAX)
#define GYRO_SCALE 
void acquisition_task() {
    int16_t acc[3], gyro[3];
    bool first = true;
    float32_t down[3];
    while (1) {
        imu_events.wait_any(EVT_FRAME_READY);
        for (int axis = 0; axis < 3; axis++) {
            read_int16(OUTX_L_XL + 2*axis, acc[axis]);
            read_int16(OUTX_L_G  + 2*axis, gyro[axis]);
        }

        float W[3];
        for (int axis = 0; axis < 3; axis++) {
            W[axis] = (250.f / I16_MAX) * gyro[axis];
            flip_buffer[flop].gyroscope[axis][i_time] = low_pass(&gyro_hist[axis], W[axis], i_time & 1);
        }

        // Rotate our previous estimation of "down" into the current device orientation
        if (!first) {
            // Convert gyroscope measurements to an axis-angle rotation
            float theta, sin_theta, cos_theta;
            arm_sqrt_f32(W[0] * W[0] + W[1] * W[1] + W[2] * W[2], &theta);
            // Normalize the axis of rotation
            for (int axis = 0; axis < 3; axis++) {
                W[axis] /= theta;
            }
            theta = -theta / POLL_RATE;

            // Rotate "down" into the current frame of reference
            arm_sin_cos_f32(theta, &sin_theta, &cos_theta);
            float WxDown[3] = cross(W, down);
            float WoDown;
            arm_dot_prod_f32(W, down, 3, &WoDown);
            for (int axis = 0; axis < 3; axis++) {
                down[axis] = (
                    down[axis] * cos_theta +
                    WxDown[axis] * sin_theta +
                    W[axis] * WoDown * (1 - cos_theta)
                );
            }
        }

        for (int axis = 0; axis < 3; axis++) {
            float raw_value = (  2.f / I16_MAX) * acc[axis];
            if (!first) {
                flip_buffer[flop].accelerometer[axis][i_time] = low_pass(&accel_hist[axis], raw_value - down[axis], i_time & 1);
                // Compensate for drift
                down[axis] = (1 - GRAVITY_DRIFT_CORRECTION) * down[axis] + GRAVITY_DRIFT_CORRECTION * raw_value;
            } else { // Assume we're stationary the first time we measure
                flip_buffer[flop].accelerometer[axis][i_time] = 0;
                down[axis] = raw_value;
            }
        }
        first = false;

        // Normalize gravity to 1g
        float down_len;
        arm_sqrt_f32(down[0] * down[0] + down[1] * down[1] + down[2] * down[2], &down_len);
        for (int axis = 0; axis < 3; axis++) {
            down[axis] = down[axis] / down_len;
        }

        #ifdef TELEPLOT
        // Print in Teleplot format (>name:value)
        printf(">acc_x:%.3f\n>acc_y:%.3f\n>acc_z:%.3f\n>gyro_x:%.2f\n>gyro_y:%.2f\n>gyro_z:%.2f\n",
            flip_buffer[flop].accelerometer[0][i_time], flip_buffer[flop].accelerometer[1][i_time], flip_buffer[flop].accelerometer[2][i_time],
            flip_buffer[flop].gyroscope[0][i_time], flip_buffer[flop].gyroscope[1][i_time], flip_buffer[flop].gyroscope[2][i_time]
        );
        #endif

        if (i_time < BATCH_SIZE_FILLED) {
            i_time += 1;
        } else {
            // Ensure the rest of the flip buffer is clear
            for (int axis = 0; axis < 3; axis++) {
                memset(&flip_buffer[flop].accelerometer[axis][BATCH_SIZE_FILLED], 0, (BATCH_SIZE - BATCH_SIZE_FILLED) * sizeof(float));
                memset(&flip_buffer[flop].gyroscope[axis][BATCH_SIZE_FILLED], 0, (BATCH_SIZE - BATCH_SIZE_FILLED) * sizeof(float));
            }
            i_time = 0;
            flop = !flop;
            // Start running all tasks that consume the data.
            // We're using a mutex so that we won't overwrite stuff that's in progress
            // and will pause measuring if necessary to prevent it. We can only safely pause here, between batches,
            // because we are only calculating FFT-related stuff within a single batch.
            #ifdef DEBUG
                if (!ingest_batch_mutex.trylock()) {
                    printf("\nIMU BUFFER OVERFLOW! Processing is taking too long!\n\n");
                    ingest_batch_mutex.lock();
                }
            #else
                ingest_batch_mutex.lock();
            #endif
            ingest_batch_condition.notify_all();
            ingest_batch_mutex.unlock();
        }
    }
}
#undef cross

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
    write_reg(CTRL2_G,   0x30); // Gyroscope:     52 Hz, ±250 dps, low pass: [polling_rate / 2]
    write_reg(CTRL1_XL,  0x30); // Accelerometer: 52 Hz, ±2 g, low pass: [polling_rate / 2]
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