/*
 * motor_control.h — MCPWM-based motor driver interface (TB6612FNG)
 *
 * Each mecanum wheel is driven by one TB6612FNG H-bridge channel.
 * Two PWM outputs per motor (IN1/IN2) control direction and speed;
 * a dedicated DIR GPIO is unused in this wiring but reserved.
 *
 * MCPWM Group 0 drives FL and FR (Operators 0 and 1).
 * MCPWM Group 1 drives RL and RR (Operators 0 and 1).
 *
 * Speed is set as a signed float in [-1.0, +1.0], mapped to duty cycle.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- GPIO pin assignments (user-configurable) ---------- */

/* Motor FL — MCPWM Group 0, Operator 0 */
#define MOTOR_FL_PWM_A_GPIO     15
#define MOTOR_FL_PWM_B_GPIO     16
#define MOTOR_FL_DIR_GPIO       17

/* Motor FR — MCPWM Group 0, Operator 1 */
#define MOTOR_FR_PWM_A_GPIO     18
#define MOTOR_FR_PWM_B_GPIO     19
#define MOTOR_FR_DIR_GPIO       20

/* Motor RL — MCPWM Group 1, Operator 0 */
#define MOTOR_RL_PWM_A_GPIO     35
#define MOTOR_RL_PWM_B_GPIO     36
#define MOTOR_RL_DIR_GPIO       37

/* Motor RR — MCPWM Group 1, Operator 1 */
#define MOTOR_RR_PWM_A_GPIO     38
#define MOTOR_RR_PWM_B_GPIO     39
#define MOTOR_RR_DIR_GPIO       40

/* TB6612FNG standby pins (active-high to enable) */
#define TB6612_STBY1_GPIO       21      /* enables FL + FR drivers   */
#define TB6612_STBY2_GPIO       45      /* enables RL + RR drivers   */

/* PWM frequency for the TB6612FNG (20 kHz keeps it above audible range) */
#define MOTOR_PWM_FREQ_HZ       20000
#define MOTOR_PWM_RESOLUTION    1000    /* timer period ticks = duty range */

/* Wheel indices — used throughout the project */
typedef enum {
    WHEEL_FL = 0,
    WHEEL_FR = 1,
    WHEEL_RL = 2,
    WHEEL_RR = 3,
    WHEEL_COUNT = 4
} wheel_id_t;

/**
 * @brief Initialise all four motor channels.
 *
 * Configures MCPWM timers, operators, comparators, and generators for each
 * motor.  Also configures TB6612FNG STBY pins as outputs and drives them HIGH.
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t motor_control_init(void);

/**
 * @brief Set the speed of one wheel.
 *
 * @param wheel  Wheel identifier (WHEEL_FL … WHEEL_RR).
 * @param speed  Signed speed in [-1.0, +1.0].  Positive = forward.
 */
void motor_set_speed(wheel_id_t wheel, float speed);

/**
 * @brief Set speed for all four wheels at once.
 *
 * @param speeds Array of 4 floats [FL, FR, RL, RR], each in [-1.0, +1.0].
 */
void motor_set_all_speeds(const float speeds[WHEEL_COUNT]);

/**
 * @brief Emergency-stop: set all motors to zero and pull STBY low.
 */
void motor_emergency_stop(void);

/**
 * @brief Re-enable motors after an emergency stop (pulls STBY high).
 */
void motor_enable(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
