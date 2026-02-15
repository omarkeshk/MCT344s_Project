/*
 * arm_kinematics.c — Forward and inverse kinematics for 4-DOF planar arm
 *
 * Kinematic model:
 *
 *   The arm has four rotary joints.  J1 rotates the entire arm about the
 *   vertical (Z) axis.  J2 and J3 form a 2-link planar manipulator in
 *   the vertical plane defined by J1.  J4 is a wrist joint that keeps
 *   the gripper pointing downward.
 *
 *   The shoulder pivot sits at height BASE_HEIGHT above the ground.
 *   Link L1 connects shoulder to elbow, L2 connects elbow to wrist,
 *   and L3 is the fixed offset from wrist to gripper tip.
 *
 *   Coordinate frame:
 *     +X = forward, +Y = left, +Z = up
 *     Origin = base rotation axis at ground level
 *
 * Convention:
 *   All angles are in degrees for the external API and converted to
 *   radians internally for trigonometric calculations.
 *
 *   Servo "0 degrees" corresponds to physical joint positions as follows:
 *     J1 = 90 deg -> arm facing forward (+X)
 *     J2 = 90 deg -> upper arm vertical
 *     J3 = 90 deg -> forearm horizontal
 *     J4 is automatically computed to keep gripper pointing down
 */

#include "arm_kinematics.h"

#include <math.h>
#include "esp_log.h"

static const char *TAG = "arm_ik";

/* ---------- Internal helpers ---------- */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static inline float deg2rad(float deg) { return deg * (M_PI / 180.0f); }
static inline float rad2deg(float rad) { return rad * (180.0f / M_PI); }

/**
 * @brief Clamp a float to [lo, hi].
 */
static inline float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* ---------- Forward Kinematics ---------- */

void arm_fk(const arm_joints_t *joints, arm_cartesian_t *pos)
{
    /* Convert servo angles to radians */
    float j1_rad = deg2rad(joints->j1);
    float j2_rad = deg2rad(joints->j2);
    float j3_rad = deg2rad(joints->j3);

    /*
     * In the vertical plane (r, z) where r is horizontal distance:
     *   The shoulder pitch j2 is measured from horizontal.
     *   The elbow pitch j3 is relative to the upper arm direction.
     *
     * Shoulder angle from horizontal = j2 (servo angle maps directly)
     * Elbow angle relative to upper arm = j3
     *
     * Position of elbow in vertical plane:
     *   r_elbow = L1 * cos(j2)
     *   z_elbow = L1 * sin(j2)
     *
     * Position of wrist relative to shoulder:
     *   r_wrist = L1*cos(j2) + L2*cos(j2 + j3)
     *   z_wrist = L1*sin(j2) + L2*sin(j2 + j3)
     *
     * The gripper extends L3 horizontally forward (when pointing down,
     * the tip is L3 further in r from the wrist).
     */

    /* Compute wrist position in vertical plane relative to shoulder */
    float r_wrist = ARM_L1 * cosf(j2_rad) + ARM_L2 * cosf(j2_rad + j3_rad);
    float z_wrist = ARM_L1 * sinf(j2_rad) + ARM_L2 * sinf(j2_rad + j3_rad);

    /* Add gripper length (assumed horizontal extension) */
    float r_total = r_wrist + ARM_L3;

    /* Convert from cylindrical (r, j1, z) to Cartesian (x, y, z) */
    pos->x = r_total * cosf(j1_rad);
    pos->y = r_total * sinf(j1_rad);
    pos->z = ARM_BASE_HEIGHT + z_wrist;
}

/* ---------- Inverse Kinematics ---------- */

bool arm_ik(const arm_cartesian_t *target, arm_joints_t *joints)
{
    float tx = target->x;
    float ty = target->y;
    float tz = target->z;

    /* --- Step 1: Base yaw (J1) from atan2(y, x) --- */
    float theta1_rad = atan2f(ty, tx);
    float theta1_deg = rad2deg(theta1_rad);

    /* Check J1 limits */
    if (theta1_deg < J1_MIN_DEG || theta1_deg > J1_MAX_DEG) {
        ESP_LOGW(TAG, "IK: J1 = %.1f deg out of range [%.0f, %.0f]",
                 theta1_deg, J1_MIN_DEG, J1_MAX_DEG);
        return false;
    }

    /* --- Step 2: Project into the vertical plane --- */
    float r_xy = sqrtf(tx * tx + ty * ty);

    /* Account for gripper length (wrist must be L3 closer to base) */
    float px = r_xy - ARM_L3;
    /* Height relative to shoulder pivot */
    float pz = tz - ARM_BASE_HEIGHT;

    /* --- Step 3: 2-link planar IK for J2 (shoulder) and J3 (elbow) --- */
    float dist_sq = px * px + pz * pz;
    float l1_sq   = ARM_L1 * ARM_L1;
    float l2_sq   = ARM_L2 * ARM_L2;

    /* Cosine of elbow angle via the law of cosines */
    float D = (dist_sq - l1_sq - l2_sq) / (2.0f * ARM_L1 * ARM_L2);

    /* Check reachability */
    if (fabsf(D) > 1.0f) {
        ESP_LOGW(TAG, "IK: target (%.1f, %.1f, %.1f) unreachable, D=%.3f",
                 tx, ty, tz, D);
        return false;
    }

    /* Elbow angle — elbow-up solution (negative sqrt for elbow above) */
    float theta3_rad = atan2f(-sqrtf(1.0f - D * D), D);

    /* Shoulder angle */
    float k1 = ARM_L1 + ARM_L2 * cosf(theta3_rad);
    float k2 = ARM_L2 * sinf(theta3_rad);
    float theta2_rad = atan2f(pz, px) - atan2f(k2, k1);

    /* --- Step 4: Wrist angle to keep gripper pointing down ---
     * The sum (theta2 + theta3) is the angle of the forearm from horizontal.
     * To point the gripper straight down, the wrist angle must compensate.
     */
    float theta4_rad = -(theta2_rad + theta3_rad);

    /* --- Convert to degrees --- */
    float theta2_deg = rad2deg(theta2_rad);
    float theta3_deg = rad2deg(theta3_rad);
    float theta4_deg = rad2deg(theta4_rad);

    /* --- Validate joint limits --- */
    if (theta2_deg < J2_MIN_DEG || theta2_deg > J2_MAX_DEG) {
        ESP_LOGW(TAG, "IK: J2 = %.1f deg out of range", theta2_deg);
        return false;
    }
    if (theta3_deg < J3_MIN_DEG || theta3_deg > J3_MAX_DEG) {
        ESP_LOGW(TAG, "IK: J3 = %.1f deg out of range", theta3_deg);
        return false;
    }
    if (theta4_deg < J4_MIN_DEG || theta4_deg > J4_MAX_DEG) {
        ESP_LOGW(TAG, "IK: J4 = %.1f deg out of range", theta4_deg);
        return false;
    }

    /* --- Output --- */
    joints->j1 = theta1_deg;
    joints->j2 = theta2_deg;
    joints->j3 = theta3_deg;
    joints->j4 = theta4_deg;

    ESP_LOGD(TAG, "IK solution: J1=%.1f J2=%.1f J3=%.1f J4=%.1f",
             joints->j1, joints->j2, joints->j3, joints->j4);
    return true;
}

/* ---------- Workspace check ---------- */

bool arm_workspace_check(const arm_cartesian_t *target)
{
    arm_joints_t dummy;
    return arm_ik(target, &dummy);
}

/* ---------- Joint-space interpolation ---------- */

void arm_joints_lerp(const arm_joints_t *from, const arm_joints_t *to,
                     float t, arm_joints_t *result)
{
    /* Clamp t to [0, 1] */
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    result->j1 = from->j1 + t * (to->j1 - from->j1);
    result->j2 = from->j2 + t * (to->j2 - from->j2);
    result->j3 = from->j3 + t * (to->j3 - from->j3);
    result->j4 = from->j4 + t * (to->j4 - from->j4);
}
