/*
 * odometry.h — Encoder reading (PCNT) and odometry computation
 *
 * Uses the ESP32-S3 hardware Pulse Counter (PCNT) peripheral in quadrature
 * decoding mode to track four wheel encoders.  Odometry is computed at 50 Hz
 * by reading accumulated counts, converting to wheel angular velocities via
 * the known encoder CPR, and applying forward kinematics + dead-reckoning
 * integration.  IMU yaw is fused with encoder-derived heading.
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Encoder GPIO assignments ---------- */
#define ENCODER_FL_A_GPIO       4
#define ENCODER_FL_B_GPIO       5
#define ENCODER_FR_A_GPIO       6
#define ENCODER_FR_B_GPIO       7
#define ENCODER_RL_A_GPIO       8
#define ENCODER_RL_B_GPIO       9
#define ENCODER_RR_A_GPIO       10
#define ENCODER_RR_B_GPIO       11

/* JGA25-371 encoder: 11 pulses/rev * 4x quadrature * 21.3:1 gear ratio
 * = 11 * 4 * 21.3 ≈ 937 counts per output-shaft revolution */
#define ENCODER_CPR             937

/* Odometry update rate */
#define ODOM_UPDATE_RATE_HZ     50

/* Complementary filter alpha for heading (IMU weight) */
#define HEADING_ALPHA           0.98f

/**
 * @brief Odometry pose estimate.
 */
typedef struct {
    float x;        /* metres, world frame          */
    float y;        /* metres, world frame          */
    float theta;    /* radians, world frame (CCW+)  */
    float vx;       /* m/s, body frame              */
    float vy;       /* m/s, body frame              */
    float omega;    /* rad/s                        */
} odom_state_t;

/**
 * @brief Initialise PCNT units for all four encoders.
 * @return ESP_OK on success.
 */
esp_err_t odometry_init(void);

/**
 * @brief Read raw encoder counts for all wheels (since last call).
 *
 * This function reads and clears the PCNT accumulators.
 *
 * @param counts Output array of 4 signed counts [FL, FR, RL, RR].
 */
void odometry_read_encoders(int32_t counts[4]);

/**
 * @brief Get the raw accumulated encoder counts (not cleared).
 *
 * @param counts Output array of 4 accumulated counts.
 */
void odometry_get_raw_counts(int32_t counts[4]);

/**
 * @brief Run one odometry update step.
 *
 * Reads encoder deltas, computes wheel velocities, applies forward kinematics,
 * fuses IMU heading, and integrates the pose.
 *
 * @param imu_yaw_rad  Current IMU yaw estimate (radians), used for heading
 *                     fusion.  Pass NAN if IMU is unavailable.
 * @param dt           Time step in seconds.
 */
void odometry_update(float imu_yaw_rad, float dt);

/**
 * @brief Get the latest odometry state (thread-safe copy).
 *
 * @param out Pointer to destination struct.
 */
void odometry_get_state(odom_state_t *out);

/**
 * @brief Reset odometry to origin (0,0,0).
 */
void odometry_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* ODOMETRY_H */
