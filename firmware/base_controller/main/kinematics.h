/*
 * kinematics.h — Mecanum wheel inverse & forward kinematics
 *
 * Coordinate frame (body-fixed):
 *   x → forward, y → left, omega → counter-clockwise
 *
 * Wheel layout (top view):
 *
 *        front
 *    FL ─────── FR
 *    |           |
 *    |     C     |
 *    |           |
 *    RL ─────── RR
 *        rear
 *
 * Inverse kinematics (body twist → wheel angular velocities):
 *   w_FL = (1/r) * ( vx - vy - (lx + ly) * omega)
 *   w_FR = (1/r) * (-vx - vy + (lx + ly) * omega)
 *   w_RL = (1/r) * (-vx - vy - (lx + ly) * omega)
 *   w_RR = (1/r) * ( vx - vy + (lx + ly) * omega)
 *
 * Forward kinematics (wheel angular velocities → body twist):
 *   vx    = (r/4) * ( w_FL - w_FR - w_RL + w_RR)
 *   vy    = (r/4) * (-w_FL - w_FR - w_RL - w_RR)
 *   omega = (r / (4*(lx+ly))) * (-w_FL + w_FR - w_RL + w_RR)
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Robot physical parameters ---------- */

/* Wheel radius in metres (JGA25-371 with 65 mm mecanum wheel) */
#define WHEEL_RADIUS_M          0.0325f

/* Half of the track width (centre-to-wheel along y axis) */
#define BASE_HALF_WIDTH_M       0.120f      /* lx */

/* Half of the wheel base (centre-to-wheel along x axis) */
#define BASE_HALF_LENGTH_M      0.135f      /* ly */

/* Derived: lx + ly */
#define LX_PLUS_LY              (BASE_HALF_WIDTH_M + BASE_HALF_LENGTH_M)

/**
 * @brief Body-frame velocity command / estimate.
 */
typedef struct {
    float vx;       /* m/s, forward positive    */
    float vy;       /* m/s, left positive        */
    float omega;    /* rad/s, CCW positive        */
} body_twist_t;

/**
 * @brief Per-wheel angular velocities (rad/s).
 */
typedef struct {
    float w[4];     /* indexed by wheel_id_t: FL, FR, RL, RR */
} wheel_speeds_t;

/**
 * @brief Inverse kinematics: body twist → wheel angular velocities.
 *
 * @param twist   Desired body-frame velocity.
 * @param out     Output wheel angular velocities (rad/s).
 */
void kinematics_inverse(const body_twist_t *twist, wheel_speeds_t *out);

/**
 * @brief Forward kinematics: wheel angular velocities → body twist.
 *
 * @param wheels  Measured wheel angular velocities (rad/s).
 * @param out     Output body-frame velocity estimate.
 */
void kinematics_forward(const wheel_speeds_t *wheels, body_twist_t *out);

/**
 * @brief Normalise wheel speeds so no wheel exceeds max_omega.
 *
 * All four speeds are scaled proportionally if any exceeds the limit.
 *
 * @param speeds     In/out wheel speeds.
 * @param max_omega  Maximum allowable angular velocity (rad/s).
 */
void kinematics_normalise(wheel_speeds_t *speeds, float max_omega);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H */
