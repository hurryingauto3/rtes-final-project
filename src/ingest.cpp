#include "ingest.hpp"
#include "conditioning.hpp"

#include "arm_math.h"

// Address info for the IMU chip
#define LSM6DSL_INT1_PIN    PD_11
#define LSM6DSL_ADDR        (0x6A << 1)

I2C i2c(PB_11, PB_10);

//MARK: Communication utilities

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

// MARK: Math functions

/** Produce rotational derivatives that move the quaternion's local axis towards the given vector.
 * Assumes acceleration and rot are both normalized.
 */
void accel_right(float accel_norm[3], float rot[4], float dest[3]) {
    // Get the global down vector in local space
    float local_z[3] = {
        2.f * (rot[3] * rot[1] + rot[0] * -rot[2]),
        2.f * (rot[3] * rot[2] + rot[0] * rot[1]),
        2.f * rot[3] * rot[3] + rot[0] * rot[0] - (rot[1] * rot[1] + rot[2] * rot[2] + rot[3] * rot[3])
    };
    // Generate a local-space rotational derivative that brings the two closer
    // Since ||cross(a, b)|| = sin(theta) and sin(theta) < theta let's just use that!
    cross(accel_norm, local_z, dest);
}

/** Approximately apply a rotational derivative to a quaternion. Dest may be rot.
 * @cite https://www.st.com/resource/en/design_tip/dt0060-exploiting-the-gyroscope-to-update-tilt-measurement-and-ecompass-stmicroelectronics.pdf
 * @param deriv Rotational derivative
*/
void rotate_quaternion(float deriv[3], float rot[4], float dest[4]) {
    float rot_prime[4] = {
        rot[0] + (- rot[1] * deriv[0] - rot[2] * deriv[1] - rot[3] * deriv[2]) / 2.f,
        rot[1] + (+ rot[0] * deriv[0] - rot[3] * deriv[1] + rot[2] * deriv[2]) / 2.f,
        rot[2] + (+ rot[3] * deriv[0] + rot[0] * deriv[1] - rot[1] * deriv[2]) / 2.f,
        rot[3] + (- rot[2] * deriv[0] + rot[1] * deriv[1] + rot[0] * deriv[2]) / 2.f
    };
    float len;
    arm_sqrt_f32(rot_prime[0] * rot_prime[0] + rot_prime[1] * rot_prime[1] + rot_prime[2] * rot_prime[2] + rot_prime[3] * rot_prime[3], &len);
    for (int axis = 0; axis < 4; axis++) {
        dest[axis] = rot_prime[axis] / len;
    }
}


/** Rotate a vector using a quaternion. Dest may be vec.
 *  @cite: https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
 */
void rotate_vector(const float vec[3], float rot[4], float dest[3]) {
    float u_fac, v_fac, ortho_fac, rot_ortho[3];

    u_fac = 2.f * (vec[0] * rot[1] + vec[1] * rot[2] + vec[2] * rot[3]);

    v_fac = rot[0] * rot[0] - (rot[1] * rot[1] + rot[2] * rot[2] + rot[3] * rot[3]);

    ortho_fac = 2.f * rot[0];
    cross(&rot[1], vec, rot_ortho);

    for (int axis = 0; axis < 3; axis++) {
        dest[axis] = (
            u_fac * rot[axis + 1]
            + v_fac * vec[axis]
            + ortho_fac * rot_ortho[axis]
        );
    }
}

void update_rot(float accel[3], float gyro[3], float rot[4]) {
    float
        accel_len, accel_norm[3], righting_deriv[3],
        rot_deriv[3] = {
            gyro[0] * (PI / (180 * POLL_RATE)),
            gyro[1] * (PI / (180 * POLL_RATE)),
            gyro[2] * (PI / (180 * POLL_RATE))};

    arm_sqrt_f32(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2], &accel_len);
    for (int axis = 0; axis < 3; axis++) { accel_norm[axis] = accel[axis] / accel_len; }

    accel_right(accel_norm, rot, righting_deriv);

    // Perform less correction when we know we're moving
    float accel_confidence = (accel_len - 1);
    accel_confidence = GRAVITY_UPDATE_RATE / (1 + MOTION_SENSITIVITY * accel_confidence * accel_confidence);
    for (int axis = 0; axis < 3; axis++) {
        rot_deriv[axis] += righting_deriv[axis] * accel_confidence;
    }

    rotate_quaternion(rot_deriv, rot, rot);
}

// MARK: Main loop

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
    float acc_f[3], gyro_f[3];
    float rot[4] = { 1, 0, 0, 0 }; // A quaternion that converts the imu-relative frame of reference to a "global" frame of reference
    FilterHistory2 acc_hist[3], gyro_hist[3]; // Low pass history
    while (1) {
        imu_events.wait_any(EVT_FRAME_READY);
        for (int axis = 0; axis < 3; axis++) {
            int16_t data;
            read_int16(OUTX_L_XL + 2*axis, data);
            acc_f[axis]  = data  * (  2.f / I16_MAX);
            read_int16(OUTX_L_G  + 2*axis, data);
            gyro_f[axis] = data * (250.f / I16_MAX);
        }

        update_rot(acc_f, gyro_f, rot);

        rotate_vector(acc_f, rot, acc_f);
        acc_f[2] -= 1;

        for (int axis = 0; axis < 3; axis++) {
            lowpass(&acc_f[axis], &acc_hist[axis], 1, i_time & 1, &flip_buffer[flop].accelerometer[axis][i_time]);
            lowpass(&gyro_f[axis], &gyro_hist[axis], 1, i_time & 1, &flip_buffer[flop].gyroscope[axis][i_time]);
        }

        #ifdef TELEPLOT
        // Print in Teleplot format (>name:value)
        // printf(">acc_x:%.3f\n>acc_y:%.3f\n>acc_z:%.3f\n>gyro_x:%.2f\n>gyro_y:%.2f\n>gyro_z:%.2f\n",
        //     flip_buffer[flop].accelerometer[0][i_time], flip_buffer[flop].accelerometer[1][i_time], flip_buffer[flop].accelerometer[2][i_time],
        //     flip_buffer[flop].gyroscope[0][i_time], flip_buffer[flop].gyroscope[1][i_time], flip_buffer[flop].gyroscope[2][i_time]
        // );
        printf(">acc_x:%3f\n>acc_y:%3f\n>acc_z:%3f\n",
            flip_buffer[flop].accelerometer[0][i_time],
            flip_buffer[flop].accelerometer[1][i_time],
            flip_buffer[flop].accelerometer[2][i_time]
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