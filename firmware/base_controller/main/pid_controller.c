/*
 * pid_controller.c — Generic discrete-time PID with anti-windup
 *
 * Implementation notes:
 *   - Derivative is computed on the measurement (not the error) to avoid
 *     derivative kick when the setpoint changes abruptly.
 *   - Integral is clamped to [integral_min, integral_max] before being
 *     added to the output (integrator anti-windup).
 *   - Final output is saturated to [out_min, out_max].
 */

#include "pid_controller.h"

/* ------------------------------------------------------------------ */
void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->kp       = kp;
    pid->ki       = ki;
    pid->kd       = kd;
    pid->out_min  = out_min;
    pid->out_max  = out_max;

    /* Default integral limits to output limits (symmetric anti-windup) */
    pid->integral_min = out_min;
    pid->integral_max = out_max;

    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first_run        = true;
}

/* ------------------------------------------------------------------ */
float pid_update(pid_ctrl_t *pid, float setpoint, float measurement, float dt)
{
    /* Guard against zero or negative dt */
    if (dt <= 0.0f) {
        return 0.0f;
    }

    float error = setpoint - measurement;

    /* --- Proportional term --- */
    float p_term = pid->kp * error;

    /* --- Integral term with clamping --- */
    pid->integral += pid->ki * error * dt;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }
    float i_term = pid->integral;

    /* --- Derivative term (derivative-on-measurement) --- */
    float d_term = 0.0f;
    if (!pid->first_run) {
        float d_measurement = (measurement - pid->prev_measurement) / dt;
        d_term = -pid->kd * d_measurement;  /* negative: counteract change */
    }
    pid->prev_measurement = measurement;
    pid->first_run = false;

    /* --- Sum and saturate --- */
    float output = p_term + i_term + d_term;
    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }

    return output;
}

/* ------------------------------------------------------------------ */
void pid_reset(pid_ctrl_t *pid)
{
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first_run        = true;
}

/* ------------------------------------------------------------------ */
void pid_set_gains(pid_ctrl_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
