/*
 * pid_controller.h — Generic discrete-time PID controller with anti-windup
 *
 * Features:
 *   - Proportional, integral, derivative terms
 *   - Integral clamping (anti-windup)
 *   - Derivative-on-measurement (avoids derivative kick on setpoint change)
 *   - Output saturation
 *   - Reset / re-initialisation
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* --- Tuning gains --- */
    float kp;               /* proportional gain                        */
    float ki;               /* integral gain                            */
    float kd;               /* derivative gain                          */

    /* --- Output limits --- */
    float out_min;           /* minimum controller output                */
    float out_max;           /* maximum controller output                */

    /* --- Anti-windup integral limits --- */
    float integral_min;      /* minimum integral accumulator value       */
    float integral_max;      /* maximum integral accumulator value       */

    /* --- Internal state (managed by the library) --- */
    float integral;          /* integral accumulator                     */
    float prev_measurement;  /* previous measurement (for D-on-meas.)   */
    bool  first_run;         /* true until first update call             */
} pid_ctrl_t;

/**
 * @brief Initialise a PID controller instance.
 *
 * @param pid        Pointer to the PID struct to initialise.
 * @param kp         Proportional gain.
 * @param ki         Integral gain.
 * @param kd         Derivative gain.
 * @param out_min    Minimum output (saturation).
 * @param out_max    Maximum output (saturation).
 */
void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd,
              float out_min, float out_max);

/**
 * @brief Compute one PID iteration.
 *
 * @param pid         Pointer to the PID struct.
 * @param setpoint    Desired value.
 * @param measurement Current measured value.
 * @param dt          Time step in seconds (must be > 0).
 * @return            Controller output, clamped to [out_min, out_max].
 */
float pid_update(pid_ctrl_t *pid, float setpoint, float measurement, float dt);

/**
 * @brief Reset the PID internal state (integral, previous error, etc.).
 */
void pid_reset(pid_ctrl_t *pid);

/**
 * @brief Update PID tuning gains at runtime.
 */
void pid_set_gains(pid_ctrl_t *pid, float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
