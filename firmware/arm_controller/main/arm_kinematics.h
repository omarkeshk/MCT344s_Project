/*
 * arm_kinematics.h — Forward and inverse kinematics for 4-DOF planar arm
 *
 * The arm is modelled as:
 *   J1 — Base yaw (rotation about Z axis)
 *   J2 — Shoulder pitch (2-link planar, link L1 = 120 mm)
 *   J3 — Elbow pitch   (2-link planar, link L2 = 120 mm)
 *   J4 — Wrist pitch   (keeps gripper pointing downward)
 *
 * The gripper extends L3 = 80 mm beyond the wrist joint.
 * The shoulder joint sits at BASE_HEIGHT = 150 mm above the ground plane.
 *
 * Coordinate frame:
 *   +X = forward (arm default reach direction)
 *   +Y = left
 *   +Z = up
 *   Origin = base rotation axis at ground level
 */

#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Arm geometry (millimetres) ---------- */
#define ARM_L1              120.0f   /* Shoulder to elbow               */
#define ARM_L2              120.0f   /* Elbow to wrist                  */
#define ARM_L3               80.0f   /* Wrist to gripper tip            */
#define ARM_BASE_HEIGHT     150.0f   /* Shoulder pivot above ground     */

/* Joint angle limits (degrees) — used for workspace checking */
#define J1_MIN_DEG            0.0f
#define J1_MAX_DEG          180.0f
#define J2_MIN_DEG            0.0f
#define J2_MAX_DEG          180.0f
#define J3_MIN_DEG            0.0f
#define J3_MAX_DEG          180.0f
#define J4_MIN_DEG            0.0f
#define J4_MAX_DEG          180.0f

/* ---------- Data types ---------- */

/* Cartesian position of the end-effector */
typedef struct {
    float x;    /* mm, forward          */
    float y;    /* mm, left             */
    float z;    /* mm, up               */
} arm_cartesian_t;

/* Joint-space representation (degrees) */
typedef struct {
    float j1;   /* Base yaw      [0, 180]  */
    float j2;   /* Shoulder      [0, 180]  */
    float j3;   /* Elbow         [0, 180]  */
    float j4;   /* Wrist         [0, 180]  */
} arm_joints_t;

/* ---------- API ---------- */

/**
 * @brief Forward kinematics: joint angles -> end-effector position.
 *
 * Joint angles are in degrees.  The returned position is in millimetres
 * relative to the base-frame origin (ground level, rotation axis).
 *
 * @param joints  Joint angles [deg].
 * @param[out] pos  Resulting Cartesian position [mm].
 */
void arm_fk(const arm_joints_t *joints, arm_cartesian_t *pos);

/**
 * @brief Inverse kinematics: target Cartesian position -> joint angles.
 *
 * Uses the decomposition:
 *   J1 = atan2(y, x)                             — base yaw
 *   J2, J3 = 2-link planar IK in the vertical plane
 *   J4 = -(J2 + J3)                              — keep gripper pointing down
 *
 * All returned angles are in degrees and clamped to [0, 180].
 *
 * @param target   Desired end-effector position [mm].
 * @param[out] joints  Computed joint angles [deg].
 * @return true if the target is reachable, false otherwise.
 */
bool arm_ik(const arm_cartesian_t *target, arm_joints_t *joints);

/**
 * @brief Check whether a Cartesian position is within the workspace.
 *
 * Performs the same math as arm_ik() but without writing results.
 *
 * @param target Position to test [mm].
 * @return true if reachable.
 */
bool arm_workspace_check(const arm_cartesian_t *target);

/**
 * @brief Linearly interpolate between two joint-space poses.
 *
 * @param from   Start pose [deg].
 * @param to     End pose [deg].
 * @param t      Interpolation parameter [0.0, 1.0].
 * @param[out] result  Interpolated pose [deg].
 */
void arm_joints_lerp(const arm_joints_t *from, const arm_joints_t *to,
                     float t, arm_joints_t *result);

#ifdef __cplusplus
}
#endif

#endif /* ARM_KINEMATICS_H */
