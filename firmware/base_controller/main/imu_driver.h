/*
 * imu_driver.h — MPU6050 I2C driver with complementary-filter yaw estimation
 *
 * Reads accelerometer and gyroscope at ~1 kHz.  Yaw (heading) is derived from
 * gyroscope integration, corrected by a complementary filter that blends in
 * the accelerometer-derived tilt (for roll/pitch) — pure yaw has no
 * magnetometer correction here, so it will drift slowly.  For a mobile robot
 * on a flat surface, gyro-integrated yaw is acceptable for short-to-medium
 * duration missions and is fused with encoder odometry elsewhere.
 */

#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- I2C Configuration ---------- */
#define IMU_I2C_SDA_GPIO        41
#define IMU_I2C_SCL_GPIO        42
#define IMU_I2C_PORT            I2C_NUM_0
#define IMU_I2C_FREQ_HZ         400000      /* 400 kHz fast mode */

/* MPU6050 I2C address (AD0 = GND) */
#define MPU6050_ADDR            0x68

/**
 * @brief IMU data packet.
 */
typedef struct {
    /* Raw sensor readings in physical units */
    float accel_x;      /* m/s^2 */
    float accel_y;      /* m/s^2 */
    float accel_z;      /* m/s^2 */
    float gyro_x;       /* rad/s */
    float gyro_y;       /* rad/s */
    float gyro_z;       /* rad/s */

    /* Filtered orientation */
    float roll;         /* radians */
    float pitch;        /* radians */
    float yaw;          /* radians (gyro-integrated, no mag correction) */
} imu_data_t;

/**
 * @brief Initialise the I2C bus and the MPU6050.
 *
 * Wakes the sensor, configures gyro +-500 deg/s, accel +-2g,
 * sets the DLPF to 42 Hz bandwidth.
 *
 * @return ESP_OK on success.
 */
esp_err_t imu_init(void);

/**
 * @brief Read accelerometer and gyroscope, update complementary filter.
 *
 * Should be called periodically (ideally at 1 kHz alongside PID).
 *
 * @param dt  Time step in seconds.
 * @return ESP_OK on success, or an I2C error code.
 */
esp_err_t imu_update(float dt);

/**
 * @brief Get the latest IMU data (thread-safe copy).
 *
 * @param out  Destination struct.
 */
void imu_get_data(imu_data_t *out);

/**
 * @brief Get the current yaw estimate in radians.
 */
float imu_get_yaw(void);

/**
 * @brief Reset the yaw integrator to zero.
 */
void imu_reset_yaw(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_DRIVER_H */
