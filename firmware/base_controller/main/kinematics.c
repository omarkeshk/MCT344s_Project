/*
 * kinematics.c — Mecanum wheel inverse & forward kinematics
 *
 * The equations follow the standard mecanum model with 45-degree rollers.
 * See kinematics.h for the full derivation and sign conventions.
 */

#include "kinematics.h"
#include <math.h>

/* ------------------------------------------------------------------ */
void kinematics_inverse(const body_twist_t *twist, wheel_speeds_t *out)
{
    float vx    = twist->vx;
    float vy    = twist->vy;
    float omega = twist->omega;
    float r_inv = 1.0f / WHEEL_RADIUS_M;

    out->w[0] = r_inv * ( vx - vy - LX_PLUS_LY * omega);   /* FL */
    out->w[1] = r_inv * (-vx - vy + LX_PLUS_LY * omega);   /* FR */
    out->w[2] = r_inv * (-vx - vy - LX_PLUS_LY * omega);   /* RL */
    out->w[3] = r_inv * ( vx - vy + LX_PLUS_LY * omega);   /* RR */
}

/* ------------------------------------------------------------------ */
void kinematics_forward(const wheel_speeds_t *wheels, body_twist_t *out)
{
    float r  = WHEEL_RADIUS_M;
    float r4 = r * 0.25f;

    float wFL = wheels->w[0];
    float wFR = wheels->w[1];
    float wRL = wheels->w[2];
    float wRR = wheels->w[3];

    out->vx    = r4 * ( wFL - wFR - wRL + wRR);
    out->vy    = r4 * (-wFL - wFR - wRL - wRR);
    out->omega = (r / (4.0f * LX_PLUS_LY)) * (-wFL + wFR - wRL + wRR);
}

/* ------------------------------------------------------------------ */
void kinematics_normalise(wheel_speeds_t *speeds, float max_omega)
{
    /* Find the maximum absolute wheel speed */
    float max_abs = 0.0f;
    for (int i = 0; i < 4; i++) {
        float a = fabsf(speeds->w[i]);
        if (a > max_abs) {
            max_abs = a;
        }
    }

    /* Scale all wheels proportionally if any exceeds the limit */
    if (max_abs > max_omega && max_abs > 0.0f) {
        float scale = max_omega / max_abs;
        for (int i = 0; i < 4; i++) {
            speeds->w[i] *= scale;
        }
    }
}
