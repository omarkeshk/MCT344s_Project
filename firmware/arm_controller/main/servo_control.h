/*
 * servo_control.h — LEDC-based servo PWM driver for 4-DOF arm + gripper
 *
 * Drives five hobby servos via the ESP32-S3 LEDC peripheral at 50 Hz.
 * Each servo maps an angle in degrees to a pulse width (500-2500 us).
 * Smooth interpolation is provided so joints can transition between
 * target angles over a configurable duration.
 *
 * Joint assignments:
 *   J1 Base Yaw      — MG996R   — LEDC Ch 0 — GPIO 15
 *   J2 Shoulder Pitch — DS3218   — LEDC Ch 1 — GPIO 16
 *   J3 Elbow Pitch    — MG996R   — LEDC Ch 2 — GPIO 17
 *   J4 Wrist Pitch    — MG90S    — LEDC Ch 3 — GPIO 18
 *   J5 Gripper        — SG90     — LEDC Ch 4 — GPIO  8
 */

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- GPIO pin assignments ---------- */
#define SERVO_J1_GPIO   15   /* Base yaw         */
#define SERVO_J2_GPIO   16   /* Shoulder pitch    */
#define SERVO_J3_GPIO   17   /* Elbow pitch       */
#define SERVO_J4_GPIO   18   /* Wrist pitch       */
#define SERVO_J5_GPIO    8   /* Gripper           */

/* ---------- LEDC channel assignments ---------- */
#define SERVO_J1_CH     0
#define SERVO_J2_CH     1
#define SERVO_J3_CH     2
#define SERVO_J4_CH     3
#define SERVO_J5_CH     4

/* ---------- PWM parameters ---------- */
#define SERVO_PWM_FREQ_HZ       50      /* Standard servo frequency         */
#define SERVO_PWM_RESOLUTION    14      /* 14-bit: 16384 ticks per 20 ms    */
#define SERVO_PWM_TICKS         16384   /* 2^14                             */
#define SERVO_PWM_PERIOD_US     20000   /* 20 ms period                     */

/* Pulse width limits (microseconds) */
#define SERVO_PULSE_MIN_US      500     /* 0 degrees                        */
#define SERVO_PULSE_MAX_US      2500    /* 180 degrees                      */

/* Angle limits (degrees) */
#define SERVO_ANGLE_MIN         0.0f
#define SERVO_ANGLE_MAX         180.0f

/* ---------- Joint count ---------- */
#define SERVO_JOINT_COUNT       5

/* Joint indices for array access */
typedef enum {
    JOINT_BASE_YAW      = 0,  /* J1 */
    JOINT_SHOULDER      = 1,  /* J2 */
    JOINT_ELBOW         = 2,  /* J3 */
    JOINT_WRIST         = 3,  /* J4 */
    JOINT_GRIPPER       = 4,  /* J5 */
} joint_id_t;

/* Per-joint angle limits (degrees). Mechanical constraints. */
typedef struct {
    float min_angle;
    float max_angle;
} servo_limits_t;

/* Interpolation state for one joint */
typedef struct {
    float start_angle;      /* angle at the start of the move         */
    float target_angle;     /* desired end angle                      */
    float current_angle;    /* current interpolated angle             */
    int64_t start_time_us;  /* timestamp when move began              */
    int64_t duration_us;    /* total duration for the move            */
    bool    moving;         /* true while interpolation is in-flight  */
} servo_interp_t;

/**
 * @brief Initialise all five LEDC channels for servo PWM output.
 *
 * Configures one LEDC timer at 50 Hz with 14-bit resolution and assigns
 * each servo to its LEDC channel and GPIO.  All servos are set to their
 * default (centre) positions on startup.
 *
 * @return ESP_OK on success, or an error code from the LEDC driver.
 */
esp_err_t servo_control_init(void);

/**
 * @brief Set a servo to an absolute angle immediately (no interpolation).
 *
 * @param joint  Joint index (JOINT_BASE_YAW .. JOINT_GRIPPER).
 * @param angle  Target angle in degrees [0, 180].  Clamped to joint limits.
 * @return ESP_OK on success.
 */
esp_err_t servo_set_angle(joint_id_t joint, float angle);

/**
 * @brief Command a smooth move to a target angle over a given duration.
 *
 * The servo_update() function must be called periodically (e.g., at 50 Hz)
 * to advance the interpolation.
 *
 * @param joint       Joint index.
 * @param target_deg  Target angle in degrees.
 * @param duration_ms Duration of the move in milliseconds (0 = instant).
 */
void servo_move_to(joint_id_t joint, float target_deg, uint32_t duration_ms);

/**
 * @brief Set target angles for joints J1-J4 simultaneously with interpolation.
 *
 * @param angles      Array of 4 floats [J1, J2, J3, J4] in degrees.
 * @param duration_ms Duration of the move in milliseconds.
 */
void servo_move_joints(const float angles[4], uint32_t duration_ms);

/**
 * @brief Advance all interpolations by one time step.
 *
 * Call this at a fixed rate (50 Hz recommended) from the servo control task.
 * Uses cosine easing for smooth acceleration/deceleration.
 */
void servo_update(void);

/**
 * @brief Check whether any joint is still moving (interpolating).
 *
 * @return true if at least one joint has not reached its target.
 */
bool servo_is_moving(void);

/**
 * @brief Read the current angle of a joint.
 *
 * @param joint Joint index.
 * @return Current angle in degrees.
 */
float servo_get_angle(joint_id_t joint);

/**
 * @brief Read current angles for all five joints.
 *
 * @param[out] angles Array of 5 floats to receive [J1..J5] in degrees.
 */
void servo_get_all_angles(float angles[SERVO_JOINT_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_CONTROL_H */
