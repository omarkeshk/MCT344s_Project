"""
Actuator Sizing Calculations — MCT344s Mobile Manipulator

Computes:
  1. Wheel motor torque requirements (mecanum drive)
  2. Arm servo torque requirements (4-DOF articulated arm)
  3. Power budget and battery runtime estimate

Week 3 deliverable for MCT344s course.
"""

import math
import numpy as np


# =============================================================================
# 1. WHEEL MOTOR TORQUE SIZING
# =============================================================================

def wheel_motor_sizing():
    """
    Calculate required torque for each mecanum wheel motor.

    Assumptions:
    - Robot moves on flat concrete floor
    - Worst case: acceleration from standstill + climbing small bump/threshold
    """
    print("=" * 60)
    print("WHEEL MOTOR TORQUE SIZING")
    print("=" * 60)

    # Robot parameters
    m_total = 5.0       # Total robot mass [kg] (conservative estimate)
    n_wheels = 4        # Number of driven wheels
    g = 9.81            # Gravitational acceleration [m/s^2]

    # Wheel parameters
    r_wheel = 0.030     # Wheel radius [m] (60mm diameter mecanum)

    # Motion parameters
    v_max = 0.5         # Max desired speed [m/s]
    a_max = 1.0         # Max desired acceleration [m/s^2]
    omega_max = 2.0     # Max angular velocity [rad/s]

    # Friction and efficiency
    mu_rolling = 0.02   # Rolling friction coefficient (rubber on concrete)
    mu_mecanum = 1.4    # Mecanum efficiency factor (rollers cause ~40% loss vs standard wheels)
    eta_gear = 0.75     # Gearbox efficiency
    eta_driver = 0.90   # Motor driver efficiency

    # Base geometry
    lx = 0.150           # Half-width [m] (center to wheel in X)
    ly = 0.150           # Half-length [m] (center to wheel in Y)

    print(f"\nRobot mass: {m_total:.1f} kg")
    print(f"Wheel radius: {r_wheel*1000:.0f} mm")
    print(f"Max speed: {v_max:.1f} m/s")
    print(f"Max acceleration: {a_max:.1f} m/s^2")

    # --- Force analysis (per wheel) ---

    # Normal force per wheel (assume equal distribution)
    F_normal = (m_total * g) / n_wheels
    print(f"\nNormal force per wheel: {F_normal:.2f} N")

    # Rolling friction force per wheel
    F_friction = mu_rolling * F_normal * mu_mecanum
    print(f"Rolling friction per wheel (with mecanum factor): {F_friction:.2f} N")

    # Acceleration force per wheel (worst case: all 4 wheels contribute equally)
    F_accel = (m_total * a_max) / n_wheels
    print(f"Acceleration force per wheel: {F_accel:.2f} N")

    # Total force per wheel (worst case: friction + acceleration)
    F_total = F_friction + F_accel
    print(f"Total force per wheel: {F_total:.2f} N")

    # --- Torque at wheel ---
    T_wheel = F_total * r_wheel
    print(f"\nTorque at wheel: {T_wheel*1000:.2f} mN-m = {T_wheel*100:.3f} N-cm")

    # Torque at motor shaft (accounting for gearbox efficiency)
    T_motor = T_wheel / eta_gear
    print(f"Torque at motor (after gearbox loss): {T_motor*1000:.2f} mN-m = {T_motor*100:.3f} N-cm")

    # --- Worst case: mecanum strafing ---
    # During pure strafing, only 2 diagonal wheels contribute effectively
    # This increases per-wheel load
    F_strafe_total = (m_total * a_max) / 2  # Only 2 effective wheels
    T_strafe_worst = (F_friction + F_strafe_total / n_wheels * 2) * r_wheel / eta_gear
    print(f"\nWorst case (strafing): {T_strafe_worst*1000:.2f} mN-m = {T_strafe_worst*100:.3f} N-cm")

    # --- Required motor speed ---
    omega_wheel_max = v_max / r_wheel  # Wheel angular velocity [rad/s]
    rpm_wheel_max = omega_wheel_max * 60 / (2 * math.pi)
    print(f"\nMax wheel speed: {omega_wheel_max:.1f} rad/s = {rpm_wheel_max:.0f} RPM")

    # During rotation, wheel speed depends on base geometry
    v_wheel_rotate = omega_max * math.sqrt(lx**2 + ly**2)
    omega_wheel_rotate = v_wheel_rotate / r_wheel
    rpm_wheel_rotate = omega_wheel_rotate * 60 / (2 * math.pi)
    print(f"Wheel speed during max rotation: {rpm_wheel_rotate:.0f} RPM")

    rpm_required = max(rpm_wheel_max, rpm_wheel_rotate)

    # --- Selected motor verification ---
    print(f"\n--- JGA25-371 Motor Verification ---")
    motor_rpm = 200         # No-load RPM at 12V (200 RPM variant)
    motor_stall_torque = 3.5  # Stall torque [kg-cm]
    motor_stall_torque_nm = motor_stall_torque * 0.0981  # Convert to N-m

    # Operating point (rule of thumb: max continuous ~30% of stall)
    motor_continuous_torque = motor_stall_torque_nm * 0.30
    print(f"Motor no-load speed: {motor_rpm} RPM")
    print(f"Motor stall torque: {motor_stall_torque} kg-cm = {motor_stall_torque_nm*1000:.0f} mN-m")
    print(f"Motor continuous torque (~30% stall): {motor_continuous_torque*1000:.0f} mN-m")
    print(f"Required torque: {T_motor*1000:.0f} mN-m")
    print(f"Required RPM: {rpm_required:.0f} RPM")

    margin_torque = motor_continuous_torque / T_motor
    margin_speed = motor_rpm / rpm_required
    print(f"\nTorque safety margin: {margin_torque:.1f}x ({'OK' if margin_torque > 1.5 else 'MARGINAL'})")
    print(f"Speed margin: {margin_speed:.1f}x ({'OK' if margin_speed > 1.2 else 'MARGINAL'})")

    return T_motor, rpm_required


# =============================================================================
# 2. ARM SERVO TORQUE SIZING
# =============================================================================

def arm_servo_sizing():
    """
    Calculate required torque for each arm joint servo.

    Static torque analysis: worst case is arm fully extended horizontally.
    """
    print("\n" + "=" * 60)
    print("ARM SERVO TORQUE SIZING")
    print("=" * 60)

    g = 9.81  # [m/s^2]

    # Link parameters
    L1 = 0.120      # Shoulder to elbow [m]
    L2 = 0.120      # Elbow to wrist [m]
    L3 = 0.080      # Wrist to gripper tip [m]

    # Mass estimates (3D printed links + servo at each joint end)
    m_link1 = 0.080   # Upper arm link mass [kg]
    m_servo2 = 0.055   # MG996R at elbow [kg]
    m_link2 = 0.060   # Forearm link mass [kg]
    m_servo3 = 0.014   # MG90S at wrist [kg]
    m_link3 = 0.040   # Wrist + gripper mechanism [kg]
    m_servo4 = 0.009   # SG90 gripper servo [kg]
    m_cube = 0.050     # Cube payload [kg] (5cm cube, ~50g estimate)

    # Base rotation (J1) — primarily overcomes friction, not gravity
    # Torque = friction from bearing + inertia during rotation
    m_arm_total = m_link1 + m_servo2 + m_link2 + m_servo3 + m_link3 + m_servo4 + m_cube
    print(f"\nTotal arm mass (with cube): {m_arm_total*1000:.0f} g")

    # --- Shoulder (J2) — worst case: arm fully horizontal ---
    print(f"\n--- Joint 2: Shoulder ---")
    # Torque = sum of (mass_i * g * distance_to_shoulder_i)
    # CG of link1 is at L1/2
    # Elbow servo is at L1
    # CG of link2 is at L1 + L2/2
    # Wrist servo is at L1 + L2
    # CG of link3 is at L1 + L2 + L3/2
    # Gripper servo at L1 + L2 + L3
    # Cube at L1 + L2 + L3

    T_shoulder = g * (
        m_link1 * (L1 / 2) +
        m_servo2 * L1 +
        m_link2 * (L1 + L2 / 2) +
        m_servo3 * (L1 + L2) +
        m_link3 * (L1 + L2 + L3 / 2) +
        m_servo4 * (L1 + L2 + L3) +
        m_cube * (L1 + L2 + L3)
    )
    T_shoulder_kgcm = T_shoulder / 0.0981
    print(f"Worst case torque (arm horizontal): {T_shoulder*1000:.1f} mN-m = {T_shoulder_kgcm:.2f} kg-cm")

    # Safety factor for dynamic loads (acceleration, vibration)
    sf = 1.5
    T_shoulder_design = T_shoulder_kgcm * sf
    print(f"Design torque (x{sf} safety factor): {T_shoulder_design:.2f} kg-cm")

    # Selected servo
    servo_shoulder = 20.0  # DS3218: 20 kg-cm
    margin = servo_shoulder / T_shoulder_design
    print(f"Selected: DS3218 (20 kg-cm)")
    print(f"Safety margin: {margin:.1f}x ({'OK' if margin > 1.3 else 'MARGINAL'})")

    # --- Elbow (J3) ---
    print(f"\n--- Joint 3: Elbow ---")
    T_elbow = g * (
        m_link2 * (L2 / 2) +
        m_servo3 * L2 +
        m_link3 * (L2 + L3 / 2) +
        m_servo4 * (L2 + L3) +
        m_cube * (L2 + L3)
    )
    T_elbow_kgcm = T_elbow / 0.0981
    T_elbow_design = T_elbow_kgcm * sf
    print(f"Worst case torque: {T_elbow*1000:.1f} mN-m = {T_elbow_kgcm:.2f} kg-cm")
    print(f"Design torque (x{sf}): {T_elbow_design:.2f} kg-cm")
    servo_elbow = 10.0  # MG996R: 10 kg-cm
    margin = servo_elbow / T_elbow_design
    print(f"Selected: MG996R (10 kg-cm)")
    print(f"Safety margin: {margin:.1f}x ({'OK' if margin > 1.3 else 'MARGINAL'})")

    # --- Wrist (J4) ---
    print(f"\n--- Joint 4: Wrist ---")
    T_wrist = g * (
        m_link3 * (L3 / 2) +
        m_servo4 * L3 +
        m_cube * L3
    )
    T_wrist_kgcm = T_wrist / 0.0981
    T_wrist_design = T_wrist_kgcm * sf
    print(f"Worst case torque: {T_wrist*1000:.1f} mN-m = {T_wrist_kgcm:.2f} kg-cm")
    print(f"Design torque (x{sf}): {T_wrist_design:.2f} kg-cm")
    servo_wrist = 2.2  # MG90S: 2.2 kg-cm
    margin = servo_wrist / T_wrist_design
    print(f"Selected: MG90S (2.2 kg-cm)")
    print(f"Safety margin: {margin:.1f}x ({'OK' if margin > 1.3 else 'MARGINAL'})")

    # --- Base Yaw (J1) ---
    print(f"\n--- Joint 1: Base Yaw ---")
    # For yaw rotation, torque = inertia * angular_accel + friction
    # Approximate: arm mass at average radius, accelerating to 90deg in 1s
    avg_radius = (L1 + L2) / 2
    I_arm = m_arm_total * avg_radius**2  # Simplified moment of inertia
    alpha_yaw = math.pi / 1.0  # 180 deg/s^2 (reach 90deg in ~1s with trapezoid profile)
    T_yaw = I_arm * alpha_yaw
    T_yaw_kgcm = T_yaw / 0.0981
    T_yaw_design = T_yaw_kgcm * sf
    print(f"Inertial torque: {T_yaw*1000:.1f} mN-m = {T_yaw_kgcm:.2f} kg-cm")
    print(f"Design torque (x{sf}): {T_yaw_design:.2f} kg-cm")
    servo_yaw = 10.0  # MG996R: 10 kg-cm
    margin = servo_yaw / T_yaw_design
    print(f"Selected: MG996R (10 kg-cm)")
    print(f"Safety margin: {margin:.1f}x ({'OK' if margin > 1.3 else 'MARGINAL'})")

    # --- Gripper (J5) ---
    print(f"\n--- Joint 5: Gripper ---")
    print(f"Gripper force requirement: hold {m_cube*1000:.0f}g cube against gravity")
    F_grip = m_cube * g * 2  # 2x safety for dynamic effects
    print(f"Required grip force: {F_grip:.2f} N")
    print(f"Selected: SG90 (1.8 kg-cm) — sufficient for lightweight gripper mechanism")

    # --- Reach verification ---
    print(f"\n--- REACH VERIFICATION ---")
    total_reach = L1 + L2 + L3
    print(f"Total arm reach from shoulder: {total_reach*1000:.0f} mm")
    base_height = 0.150  # Shoulder height above base frame [m]
    pedestal_height = 0.400  # Competition pedestal [m]
    vertical_needed = pedestal_height - base_height
    print(f"Base to shoulder height: {base_height*1000:.0f} mm")
    print(f"Pedestal height: {pedestal_height*1000:.0f} mm")
    print(f"Vertical reach needed: {vertical_needed*1000:.0f} mm")
    print(f"Arm reach: {total_reach*1000:.0f} mm")
    reach_margin = total_reach / vertical_needed
    print(f"Reach margin: {reach_margin:.2f}x ({'OK' if reach_margin > 1.2 else 'MARGINAL'})")

    return T_shoulder_kgcm, T_elbow_kgcm, T_wrist_kgcm


# =============================================================================
# 3. POWER BUDGET AND BATTERY SIZING
# =============================================================================

def power_budget():
    """Calculate power consumption and battery runtime."""
    print("\n" + "=" * 60)
    print("POWER BUDGET AND BATTERY SIZING")
    print("=" * 60)

    # Battery specs
    V_batt = 11.1    # 3S LiPo nominal [V]
    C_batt = 5.0     # Battery capacity [Ah]
    E_batt = V_batt * C_batt  # Total energy [Wh]

    print(f"\nBattery: 3S LiPo, {V_batt:.1f}V, {C_batt:.0f}Ah ({E_batt:.1f} Wh)")

    # Power consumers
    consumers = [
        ("4x DC Motors (avg driving)", 12.0, 2.0),     # V, A
        ("4x DC Motors (peak accel)", 12.0, 4.0),
        ("5x Servos (avg moving)", 5.0, 0.5),
        ("5x Servos (peak)", 5.0, 2.0),
        ("Raspberry Pi 4B", 5.0, 1.0),
        ("2x ESP32-S3", 5.0, 0.3),
        ("Motor drivers + misc", 5.0, 0.2),
    ]

    print(f"\n{'Consumer':<35} {'Voltage':>8} {'Current':>8} {'Power':>8}")
    print("-" * 61)

    total_avg = 0
    total_peak = 0
    avg_items = [0, 2, 4, 5, 6]     # Average scenario indices
    peak_items = [1, 3, 4, 5, 6]    # Peak scenario indices

    for i, (name, v, a) in enumerate(consumers):
        p = v * a
        scenario = "avg" if i in avg_items else "peak"
        print(f"  {name:<33} {v:>6.1f}V {a:>6.2f}A {p:>6.1f}W  [{scenario}]")

    # Calculate totals
    P_avg = sum(v * a for i, (_, v, a) in enumerate(consumers) if i in avg_items)
    P_peak = sum(v * a for i, (_, v, a) in enumerate(consumers) if i in peak_items)
    I_avg = P_avg / V_batt
    I_peak = P_peak / V_batt

    print(f"\n{'Total average power:':<35} {P_avg:>24.1f}W")
    print(f"{'Total peak power:':<35} {P_peak:>24.1f}W")
    print(f"{'Average current from battery:':<35} {I_avg:>23.2f}A")
    print(f"{'Peak current from battery:':<35} {I_peak:>23.2f}A")

    # Runtime
    runtime_avg_h = C_batt / I_avg
    runtime_avg_min = runtime_avg_h * 60
    print(f"\n{'Estimated runtime (avg load):':<35} {runtime_avg_h:>20.1f} hours ({runtime_avg_min:.0f} min)")
    print(f"{'Competition run time:':<35} {'5 min':>24}")
    print(f"{'Runtime margin:':<35} {runtime_avg_min/5:>23.0f}x")

    # Fuse sizing
    fuse_rating = math.ceil(I_peak * 1.5)  # 1.5x peak for fuse rating
    print(f"\n{'Recommended fuse rating:':<35} {fuse_rating:>23}A")

    # Buck converter requirements
    I_5v_total = sum(a for i, (_, v, a) in enumerate(consumers)
                     if v == 5.0 and i in avg_items)
    print(f"\n{'Total 5V rail current (avg):':<35} {I_5v_total:>22.1f}A")
    print(f"{'RPi buck converter:':<35} {'5V / 3A (MP1584 or LM2596)':>24}")
    print(f"{'Servo buck converter:':<35} {'5V / 3A (separate rail)':>24}")


# =============================================================================
# 4. KINEMATICS VERIFICATION
# =============================================================================

def kinematics_verification():
    """Verify mecanum kinematics and arm IK with test cases."""
    print("\n" + "=" * 60)
    print("KINEMATICS VERIFICATION")
    print("=" * 60)

    # Mecanum parameters
    r = 0.030    # Wheel radius [m]
    lx = 0.150   # Half-width [m]
    ly = 0.150   # Half-length [m]

    def mecanum_ik(vx, vy, omega):
        """Inverse kinematics: body velocity -> wheel speeds [rad/s]."""
        w1 = (1/r) * ( vx - vy - (lx + ly) * omega)  # FL
        w2 = (1/r) * (-vx - vy + (lx + ly) * omega)  # FR
        w3 = (1/r) * (-vx - vy - (lx + ly) * omega)  # RL
        w4 = (1/r) * ( vx - vy + (lx + ly) * omega)  # RR
        return w1, w2, w3, w4

    def mecanum_fk(w1, w2, w3, w4):
        """Forward kinematics: wheel speeds -> body velocity."""
        vx = (r/4) * ( w1 - w2 - w3 + w4)
        vy = (r/4) * (-w1 - w2 - w3 - w4)
        omega = (r / (4*(lx+ly))) * (-w1 + w2 - w3 + w4)
        return vx, vy, omega

    # Test cases
    test_cases = [
        ("Pure Y (forward 0.3 m/s)", 0.0, 0.3, 0.0),
        ("Pure X (strafe right 0.3 m/s)", 0.3, 0.0, 0.0),
        ("Pure rotation (1 rad/s CCW)", 0.0, 0.0, 1.0),
        ("Diagonal (0.2, 0.2, 0)", 0.2, 0.2, 0.0),
        ("Combined (0.1, 0.2, 0.5)", 0.1, 0.2, 0.5),
    ]

    print(f"\n{'Test Case':<35} {'vx':>6} {'vy':>6} {'ω':>6} | {'w1':>7} {'w2':>7} {'w3':>7} {'w4':>7} | FK OK?")
    print("-" * 100)

    for name, vx, vy, omega in test_cases:
        w1, w2, w3, w4 = mecanum_ik(vx, vy, omega)
        vx_fk, vy_fk, omega_fk = mecanum_fk(w1, w2, w3, w4)

        # Verify FK recovers IK input
        ok = (abs(vx - vx_fk) < 1e-6 and
              abs(vy - vy_fk) < 1e-6 and
              abs(omega - omega_fk) < 1e-6)

        print(f"  {name:<33} {vx:>6.2f} {vy:>6.2f} {omega:>6.2f} | "
              f"{w1:>7.1f} {w2:>7.1f} {w3:>7.1f} {w4:>7.1f} | {'PASS' if ok else 'FAIL'}")

    # --- Arm IK verification ---
    print(f"\n--- Arm Inverse Kinematics Verification ---")
    L1 = 0.120
    L2 = 0.120
    L3 = 0.080
    base_h = 0.150

    def arm_ik(target_x, target_y, target_z):
        """Solve arm IK for target in base frame (gripper pointing down)."""
        theta1 = math.atan2(target_y, target_x)

        r_xy = math.sqrt(target_x**2 + target_y**2)
        # Gripper points downward: L3 is subtracted from Z, not from horizontal
        pz = target_z - base_h - L3
        px = r_xy

        D = (px**2 + pz**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if abs(D) > 1.0:
            return None  # Out of reach

        theta3 = math.atan2(-math.sqrt(1 - D**2), D)  # Elbow-up
        k1 = L1 + L2 * math.cos(theta3)
        k2 = L2 * math.sin(theta3)
        theta2 = math.atan2(pz, px) - math.atan2(k2, k1)
        theta4 = -(theta2 + theta3)  # Keep gripper pointing down

        return (math.degrees(theta1), math.degrees(theta2),
                math.degrees(theta3), math.degrees(theta4))

    arm_tests = [
        ("Pedestal (front, 5cm away)", 0.05, 0.0, 0.40),
        ("Pedestal (front, 10cm away)", 0.10, 0.0, 0.40),
        ("Pedestal (front, 15cm away)", 0.15, 0.0, 0.40),
        ("Pedestal (side, 10cm away)", 0.0, 0.10, 0.40),
        ("Low storage bin", 0.15, 0.0, 0.20),
        ("Ground drop-off (15cm fwd)", 0.15, 0.0, 0.10),
        ("Above pedestal (10cm fwd)", 0.10, 0.0, 0.45),
    ]

    print(f"\n{'Test Case':<30} {'x':>6} {'y':>6} {'z':>6} | {'J1':>7} {'J2':>7} {'J3':>7} {'J4':>7} | Result")
    print("-" * 95)

    for name, x, y, z in arm_tests:
        result = arm_ik(x, y, z)
        if result:
            j1, j2, j3, j4 = result
            print(f"  {name:<28} {x:>6.2f} {y:>6.2f} {z:>6.2f} | "
                  f"{j1:>7.1f} {j2:>7.1f} {j3:>7.1f} {j4:>7.1f} | REACHABLE")
        else:
            print(f"  {name:<28} {x:>6.2f} {y:>6.2f} {z:>6.2f} | "
                  f"{'---':>7} {'---':>7} {'---':>7} {'---':>7} | OUT OF REACH")


# =============================================================================
# MAIN
# =============================================================================

if __name__ == '__main__':
    print("MCT344s — Actuator Sizing Report")
    print("Modular Omni-Wheel Mobile Manipulator")
    print(f"{'=' * 60}\n")

    wheel_motor_sizing()
    arm_servo_sizing()
    power_budget()
    kinematics_verification()

    print(f"\n{'=' * 60}")
    print("END OF REPORT")
    print(f"{'=' * 60}")
