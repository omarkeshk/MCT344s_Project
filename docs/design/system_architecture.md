# System Architecture — Modular Omni-Wheel Mobile Manipulator

**VDI 2206 Phase:** Phase 1 — System-Level Design

---

## 1. Architecture Overview

The system follows a three-layer hybrid architecture with two real-time modules and one supervisory controller.

```
┌─────────────────────────────────────────────────────────┐
│                  SUPERVISORY LAYER                       │
│                  (Raspberry Pi 4B)                        │
│                                                          │
│  ┌──────────┐ ┌───────────┐ ┌─────────┐ ┌────────────┐ │
│  │ Teleop   │ │ Perception│ │ Mission │ │ Navigation │ │
│  │ Handler  │ │ Pipeline  │ │ Planner │ │ Controller │ │
│  └────┬─────┘ └─────┬─────┘ └────┬────┘ └──────┬─────┘ │
│       └──────┬──────┴─────┬──────┘             │       │
│              │            │                    │       │
│         UART/USB     UART/USB             Camera (CSI) │
└──────────┬───────────┬──────────────────────────────────┘
           │           │
    ┌──────┴──────┐  ┌─┴────────────┐
    │ MOBILE BASE │  │ MANIPULATOR  │
    │ MODULE      │  │ MODULE       │
    │ (ESP32-S3)  │  │ (ESP32-S3)   │
    │             │  │              │
    │ • Motor PID │  │ • Servo PWM  │
    │ • Encoders  │  │ • Arm IK     │
    │ • IMU       │  │ • Gripper    │
    │ • Odometry  │  │              │
    └──────┬──────┘  └──────┬───────┘
           │                │
           └──── CAN Bus ───┘
          (Inter-module link)
```

---

## 2. Module Specifications

### 2.1 Mobile Base Module

**Controller:** ESP32-S3-DevKitC-1 (N16R8)
**Firmware framework:** ESP-IDF v5.x (C, FreeRTOS)

| Subsystem | Hardware | ESP32 Peripheral | Rate |
|-----------|----------|-----------------|------|
| Motor 1 (FL) | JGA25-371 + TB6612FNG | MCPWM Group 0, Operator 0 | PWM @ 20 kHz |
| Motor 2 (FR) | JGA25-371 + TB6612FNG | MCPWM Group 0, Operator 1 | PWM @ 20 kHz |
| Motor 3 (RL) | JGA25-371 + TB6612FNG | MCPWM Group 1, Operator 0 | PWM @ 20 kHz |
| Motor 4 (RR) | JGA25-371 + TB6612FNG | MCPWM Group 1, Operator 1 | PWM @ 20 kHz |
| Encoder 1 (FL) | Built-in hall-effect | PCNT Unit 0 | Hardware counting |
| Encoder 2 (FR) | Built-in hall-effect | PCNT Unit 1 | Hardware counting |
| Encoder 3 (RL) | Built-in hall-effect | PCNT Unit 2 | Hardware counting |
| Encoder 4 (RR) | Built-in hall-effect | PCNT Unit 3 | Hardware counting |
| IMU | MPU6050 | I2C (SDA/SCL) | 100 Hz read |
| CAN transceiver | SN65HVD230 | TWAI (TX/RX) | 500 kbps |
| UART to RPi | USB-UART bridge | UART0 (USB) | 921600 baud |
| Battery monitor | Voltage divider | ADC1 Channel | 1 Hz read |

**FreeRTOS Task Architecture:**

| Task | Priority | Rate | Function |
|------|----------|------|----------|
| Motor Control | Highest | 1 kHz | Read encoders, compute PID, output PWM |
| Odometry | High | 50 Hz | Compute (x, y, θ) from encoders + IMU |
| Communication | Medium | 100 Hz | UART RX/TX to RPi, CAN RX/TX to arm |
| Safety Monitor | Medium | 10 Hz | Heartbeat check, watchdog, battery level |

### 2.2 Manipulator Module

**Controller:** ESP32-S3-DevKitC-1 (N8R2)
**Firmware framework:** ESP-IDF v5.x (C, FreeRTOS)

| Joint | Servo | ESP32 Peripheral | Range |
|-------|-------|-----------------|-------|
| J1 — Base Yaw | MG996R (10 kg-cm) | LEDC Channel 0 | 0°–180° |
| J2 — Shoulder Pitch | DS3218 (20 kg-cm) | LEDC Channel 1 | 0°–180° |
| J3 — Elbow Pitch | MG996R (10 kg-cm) | LEDC Channel 2 | 0°–180° |
| J4 — Wrist Pitch | MG90S (2.2 kg-cm) | LEDC Channel 3 | 0°–180° |
| J5 — Gripper | SG90 (1.8 kg-cm) | LEDC Channel 4 | Open/Close |

**Link Lengths:**
- L1 (shoulder to elbow): 120 mm
- L2 (elbow to wrist): 120 mm
- L3 (wrist to gripper tip): 80 mm
- Base height (frame top to shoulder axis): ~150 mm
- Total reach: ~320 mm from shoulder axis
- Required vertical reach: 400 mm (pedestal) - 150 mm (base height) = 250 mm — within 320 mm envelope

**FreeRTOS Task Architecture:**

| Task | Priority | Rate | Function |
|------|----------|------|----------|
| Servo Control | Highest | 50 Hz | Interpolate to target angles, output PWM |
| IK Solver | High | 20 Hz | Compute joint angles from Cartesian target |
| Communication | Medium | 100 Hz | UART to RPi, CAN to base |
| Gripper | Low | 10 Hz | Monitor grasp state |

### 2.3 Supervisory Controller (Raspberry Pi 4B)

**OS:** Raspberry Pi OS (64-bit)
**Language:** Python 3.10+
**Key libraries:** OpenCV, pyzbar, numpy, pyserial, pygame

| Subsystem | Description | Rate |
|-----------|-------------|------|
| Camera Driver | Capture frames from CSI/USB camera | 30 fps |
| QR Detector | Decode QR codes using pyzbar | Per frame |
| Color Classifier | HSV thresholding for R/G/B detection | Per frame |
| Mission Planner | Finite state machine for autonomous sequence | 10 Hz |
| Navigation | Waypoint following using odometry feedback | 10 Hz |
| Teleop Handler | Read gamepad input via pygame | 50 Hz |
| Serial Bridge | UART communication to both ESP32s | 100 Hz |
| Telemetry Display | Show encoder/IMU/color data on screen | 10 Hz |

---

## 3. Communication Architecture

### 3.1 Channel Overview

| Channel | Physical | Protocol | Bandwidth | Purpose |
|---------|----------|----------|-----------|---------|
| RPi ↔ Base ESP32 | USB cable | UART 921600 baud | ~92 KB/s | Velocity commands, odometry |
| RPi ↔ Arm ESP32 | USB cable | UART 921600 baud | ~92 KB/s | Joint commands, arm status |
| Base ↔ Arm ESP32 | Twisted pair (via connector) | CAN 2.0B 500kbps | ~50 KB/s | Heartbeat, ESTOP, coordination |

### 3.2 UART Message Frame Format

```
┌───────┬────────┬────────┬─────────────────┬──────┬─────┐
│ START │ MSG_ID │ LENGTH │    PAYLOAD      │ CRC8 │ END │
│ 0xAA  │  1B    │  1B    │   0-255 bytes   │  1B  │0x55 │
└───────┴────────┴────────┴─────────────────┴──────┴─────┘
```

- **START:** 0xAA (sync byte)
- **MSG_ID:** Message type identifier
- **LENGTH:** Payload length in bytes (0-255)
- **PAYLOAD:** Message-specific data (little-endian)
- **CRC8:** CRC-8/MAXIM over MSG_ID + LENGTH + PAYLOAD
- **END:** 0x55 (end byte)

### 3.3 Message Definitions

| ID | Name | Direction | Payload | Size |
|----|------|-----------|---------|------|
| 0x01 | CMD_VELOCITY | RPi → Base | vx(f32) vy(f32) omega(f32) | 12B |
| 0x02 | ODOMETRY | Base → RPi | x(f32) y(f32) theta(f32) vx(f32) vy(f32) | 20B |
| 0x03 | CMD_ARM_JOINTS | RPi → Arm | j1(f32) j2(f32) j3(f32) j4(f32) | 16B |
| 0x04 | CMD_GRIPPER | RPi → Arm | state(u8): 0=open 1=close | 1B |
| 0x05 | ARM_STATUS | Arm → RPi | j1-j5(5xf32) gripper(u8) | 21B |
| 0x06 | MODE_SWITCH | RPi → Both | mode(u8): 0=TELEOP 1=AUTO 2=ESTOP | 1B |
| 0x07 | HEARTBEAT | Bidirectional | timestamp(u32) status(u8) | 5B |
| 0x08 | SENSOR_DATA | Base → RPi | imu_yaw(f32) enc1-4(4xi32) | 20B |
| 0x09 | CMD_ARM_CART | RPi → Arm | x(f32) y(f32) z(f32) | 12B |
| 0x0A | ERROR | Any → RPi | error_code(u8) details(str) | Variable |

### 3.4 CAN Bus Messages (Base ↔ Arm)

| CAN ID | Name | Direction | Data (8B max) |
|--------|------|-----------|---------------|
| 0x100 | BASE_HEARTBEAT | Base → Arm | timestamp(u32) mode(u8) battery(u8) |
| 0x200 | ARM_HEARTBEAT | Arm → Base | timestamp(u32) state(u8) |
| 0x101 | BASE_ESTOP | Base → Arm | estop_active(u8) |
| 0x201 | ARM_ESTOP | Arm → Base | estop_active(u8) |
| 0x102 | BASE_MOTION_STATE | Base → Arm | vx(i16) vy(i16) omega(i16) |
| 0x202 | ARM_BUSY | Arm → Base | busy(u8) — arm in motion, don't move base |

### 3.5 Safety Protocol

1. **Heartbeat:** Each module sends heartbeat at ≥2 Hz
2. **Timeout:** If no heartbeat received for 500 ms → enter safe stop state
3. **ESTOP:** MODE_SWITCH with mode=2 → all actuators halt immediately
4. **Watchdog:** If base receives no CMD_VELOCITY for 500 ms → stop all motors
5. **CAN ESTOP:** Hardware ESTOP pin asserted → CAN ESTOP message broadcasted → all modules halt

---

## 4. Modular Interface Specification

### 4.1 Mechanical Interface

```
    ┌─────────────────────────┐
    │    ARM MODULE           │
    │    MOUNTING PLATE       │
    │                         │
    │  ○ (pin)    ○ (pin)    │     ○ = ø4mm dowel pin
    │                         │     ● = M4 bolt hole
    │  ●          ●           │
    │      80mm               │
    │  ●          ●           │
    │      80mm               │
    │                         │
    └─────────────────────────┘
```

- **Bolt pattern:** 4x M4 through-holes, 80 mm x 80 mm square
- **Alignment:** 2x ø4 mm dowel pins for repeatable positioning
- **Surface:** Flat machined aluminum plate (3 mm minimum)
- **Attachment:** Thumb screws or wing nuts for tool-free detachment

### 4.2 Electrical Interface

**Connector:** GX16-6 (6-pin keyed aviation connector)

| Pin | Signal | Wire Gauge | Color Code |
|-----|--------|-----------|------------|
| 1 | CAN_H | 22 AWG | Blue |
| 2 | CAN_L | 22 AWG | Blue/White |
| 3 | +12V_RAW | 18 AWG | Red |
| 4 | GND | 18 AWG | Black |
| 5 | ESTOP | 22 AWG | Yellow |
| 6 | +5V_SERVO | 20 AWG | Orange |

**Notes:**
- CAN_H and CAN_L should be a twisted pair for noise immunity
- +12V_RAW provides unregulated battery voltage (11.1V nominal)
- ESTOP is active-low, open-drain (pulled up to 3.3V on each module)
- +5V_SERVO carries regulated 5V from base buck converter for arm servos

---

## 5. Power Architecture

```
                    ┌──────────────┐
                    │  3S LiPo     │
                    │  11.1V       │
                    │  5000 mAh    │
                    └──────┬───────┘
                           │
                    ┌──────┴───────┐
                    │  Main Switch │
                    │  + 15A Fuse  │
                    └──────┬───────┘
                           │
              ┌────────────┼────────────┐
              │            │            │
        ┌─────┴─────┐ ┌───┴────┐ ┌─────┴──────┐
        │ TB6612FNG │ │ Buck   │ │ Buck       │
        │ Motor     │ │ 5V/3A  │ │ 5V/3A      │
        │ Drivers   │ │ (RPi)  │ │ (Servos)   │
        │ (12V in)  │ └───┬────┘ └─────┬──────┘
        └─────┬─────┘     │            │
              │        ┌───┴────┐  ┌───┴────────┐
        ┌─────┴─────┐  │  RPi   │  │ GX16-6     │
        │ 4x Motors │  │  4B    │  │ Connector  │
        │ JGA25-371 │  └───┬────┘  │ to Arm     │
        └───────────┘      │       └────────────┘
                     ┌─────┴──────┐
                     │ 2x ESP32   │
                     │ (USB from  │
                     │  RPi 5V)   │
                     └────────────┘
```

**Power budget:**

| Consumer | Voltage | Current (avg) | Current (peak) |
|----------|---------|--------------|----------------|
| 4x DC motors | 11.1V | 2.0A | 6.0A (stall) |
| 5x Servos | 5V | 0.5A | 2.5A |
| Raspberry Pi 4B | 5V | 1.0A | 1.5A |
| 2x ESP32-S3 | 5V (USB) | 0.3A | 0.5A |
| Motor drivers, misc | 5V | 0.1A | 0.2A |
| **Total** | — | **3.9A** | **10.7A** |

**Runtime estimate:** 5000 mAh / 3900 mA ≈ 1.3 hours (competition needs 5 minutes)

---

## 6. Mecanum Wheel Kinematics

### Motor Numbering Convention
```
      FRONT
  ┌─────────────┐
  │ M1(FL) M2(FR)│    Robot body (top view)
  │              │    Arrow = forward (Y+)
  │      →X+     │
  │              │
  │ M3(RL) M4(RR)│
  └─────────────┘
      REAR
```

### Inverse Kinematics (body velocity → wheel speeds)

Given: vx (strafe), vy (forward), ω (rotation), wheel radius r, half-width lx, half-length ly:

```
ω1 = (1/r) * ( vx - vy - (lx + ly) * ω)    // Front-Left
ω2 = (1/r) * (-vx - vy + (lx + ly) * ω)    // Front-Right
ω3 = (1/r) * (-vx - vy - (lx + ly) * ω)    // Rear-Left
ω4 = (1/r) * ( vx - vy + (lx + ly) * ω)    // Rear-Right
```

### Forward Kinematics (wheel speeds → body velocity)

```
vx    = (r/4) * ( ω1 - ω2 - ω3 + ω4)
vy    = (r/4) * (-ω1 - ω2 - ω3 - ω4)
omega = (r / (4*(lx+ly))) * (-ω1 + ω2 - ω3 + ω4)
```

### Competition Maneuvers

| Maneuver | vx | vy | ω | Wheel Pattern |
|----------|----|----|---|---------------|
| Pure Y (Red station) | 0 | V | 0 | All wheels same speed, same direction |
| Pure X (Blue station) | V | 0 | 0 | Diagonal pairs opposite directions |
| Rotate (Green station) | 0 | 0 | W | Left wheels vs right wheels opposite |

---

## 7. Arm Inverse Kinematics

### Decomposition
1. **Base yaw (J1):** θ1 = atan2(target_y, target_x) in base frame
2. **Shoulder + Elbow (J2, J3):** 2-link planar IK in vertical plane
3. **Wrist (J4):** Set to maintain downward gripper orientation

### 2-Link Planar IK (Shoulder and Elbow)

Given target position (px, pz) in the arm's vertical plane, link lengths L1 and L2:

```
D = (px² + pz² - L1² - L2²) / (2 * L1 * L2)
if |D| > 1: target is out of reach

θ_elbow = atan2(-√(1 - D²), D)         // Elbow-up solution
k1 = L1 + L2 * cos(θ_elbow)
k2 = L2 * sin(θ_elbow)
θ_shoulder = atan2(pz, px) - atan2(k2, k1)
θ_wrist = desired_angle - θ_shoulder - θ_elbow   // Keep gripper pointing down
```

---

## 8. Autonomous Mission State Machine

```
    ┌──────┐
    │ IDLE │
    └──┬───┘
       │ start autonomous
       ▼
  ┌────────────┐
  │ SCAN_CUBE  │◄─────────────┐
  └────┬───────┘              │
       │ cube detected        │
       ▼                      │
  ┌────────────┐              │
  │ APPROACH   │              │
  └────┬───────┘              │
       │ in position          │
       ▼                      │
  ┌────────────┐              │
  │ READ_QR    │              │
  └────┬───────┘              │
       │ color decoded        │
       ▼                      │
  ┌────────────┐              │
  │ PICK_UP    │              │
  └────┬───────┘              │
       │ cube gripped         │
       ▼                      │
  ┌────────────┐              │
  │ STORE_CUBE │              │
  └────┬───────┘              │
       │ in bin               │
       ▼                      │
  ┌────────────┐              │
  │ CHECK_DONE │──── more ────┘
  └────┬───────┘
       │ all done
       ▼
  ┌────────────┐
  │ COMPLETE   │
  └────────────┘
```

After picking, the operator manually drives to the correct station, then the constrained maneuver (Pure X/Y/Rotate) is executed with encoder + IMU feedback.
