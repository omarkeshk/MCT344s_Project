/*
 * gripper.h — Gripper control for SG90 servo (J5)
 *
 * Provides open/close commands with position presets and a simple
 * timeout-based grasp detection.  When the gripper is commanded to
 * close, it moves incrementally.  If it stalls (fails to reach the
 * fully-closed angle within the timeout), we infer an object is grasped.
 */

#ifndef GRIPPER_H
#define GRIPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Gripper angle presets (degrees) ---------- */
#define GRIPPER_OPEN_ANGLE      120.0f   /* Fully open                */
#define GRIPPER_CLOSE_ANGLE      30.0f   /* Fully closed (no object)  */
#define GRIPPER_HALF_ANGLE       75.0f   /* Half-open                 */

/* ---------- Grasp detection parameters ---------- */
#define GRIPPER_CLOSE_STEP_DEG    2.0f   /* Increment per update step */
#define GRIPPER_CLOSE_TIMEOUT_MS 2000    /* Max time to reach close   */
#define GRIPPER_STALL_THRESH_DEG  5.0f   /* If final angle > close + this, grasped */

/* ---------- Gripper states ---------- */
typedef enum {
    GRIPPER_STATE_OPEN      = 0,
    GRIPPER_STATE_CLOSED    = 1,
    GRIPPER_STATE_CLOSING   = 2,
    GRIPPER_STATE_GRASPED   = 3,
    GRIPPER_STATE_OPENING   = 4,
} gripper_state_t;

/**
 * @brief Initialise the gripper module.
 *
 * Sets the gripper servo to the open position.  Must be called after
 * servo_control_init().
 *
 * @return ESP_OK on success.
 */
esp_err_t gripper_init(void);

/**
 * @brief Command the gripper to open.
 *
 * Moves to GRIPPER_OPEN_ANGLE over the default interpolation period.
 */
void gripper_open(void);

/**
 * @brief Command the gripper to close.
 *
 * Initiates an incremental close sequence with grasp detection.
 */
void gripper_close(void);

/**
 * @brief Set the gripper to a specific angle.
 *
 * @param angle_deg Target angle in degrees.
 */
void gripper_set_angle(float angle_deg);

/**
 * @brief Periodic update for the gripper state machine.
 *
 * Call this at ~10 Hz from the gripper task.  Handles the incremental
 * closing sequence and grasp detection timeout.
 */
void gripper_update(void);

/**
 * @brief Get the current gripper state.
 *
 * @return Current gripper_state_t.
 */
gripper_state_t gripper_get_state(void);

/**
 * @brief Check whether an object is currently grasped.
 *
 * @return true if gripper_state == GRIPPER_STATE_GRASPED.
 */
bool gripper_is_grasped(void);

/**
 * @brief Get the current gripper angle.
 *
 * @return Angle in degrees.
 */
float gripper_get_angle(void);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_H */
