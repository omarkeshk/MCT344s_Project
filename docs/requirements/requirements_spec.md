# Requirements Specification — Modular Omni-Wheel Mobile Manipulator

**Course:** MCT333/MCT344 — Mechatronic Systems Design (Spring 2026)
**Document Version:** 1.0
**VDI 2206 Phase:** Phase 0 — Problem Definition and Requirements

---

## 1. Introduction

### 1.1 Purpose
This document defines the system-level requirements for the Modular Omni-Wheel Mobile Manipulator robot, designed for the Spring 2026 Mechatronics Omni-Challenge competition.

### 1.2 Scope
The system comprises a holonomic mobile base and a robotic manipulator arm with gripper, operating as two plug-and-play modules. The robot must navigate a competition track under manual teleoperation, then autonomously pick up QR-coded cubes, sort them by color, and deliver them to matching drop-off zones.

### 1.3 Definitions and Abbreviations

| Term | Definition |
|------|------------|
| DOF | Degrees of Freedom |
| IK | Inverse Kinematics |
| FK | Forward Kinematics |
| IMU | Inertial Measurement Unit |
| QR | Quick Response (code) |
| PID | Proportional-Integral-Derivative controller |
| CAN | Controller Area Network |
| UART | Universal Asynchronous Receiver-Transmitter |
| MCPWM | Motor Control PWM |
| PCNT | Pulse Counter |
| LiPo | Lithium Polymer (battery) |
| TWAI | Two-Wire Automotive Interface (ESP32 CAN) |
| ESTOP | Emergency Stop |

---

## 2. Stakeholders

| Stakeholder | Interest |
|-------------|----------|
| Competition organizers / instructors | Rule compliance, safety, scoring |
| Team members (7-8 students) | Feasibility, maintainability, clear interfaces |
| Judges / TAs | Documentation quality, safety, individual contribution |
| University lab | Manufacturing constraints, equipment availability |

---

## 3. Functional Requirements

| ID | Requirement | Priority | Acceptance Criteria |
|----|------------|----------|---------------------|
| FR-01 | The mobile base SHALL provide holonomic motion (translation in X, Y and rotation about center) using mecanum wheels. | Critical | Robot can translate in any direction and rotate independently, demonstrated on flat surface. |
| FR-02 | The robot SHALL be manually controllable via wireless teleoperation during the Manual Control Zone. | Critical | Operator controls robot with gamepad from >10 m distance with <200 ms latency. |
| FR-03 | The robotic arm SHALL grasp cubes (5 cm x 5 cm x 5 cm) from pedestals at approximately 40 cm height. | Critical | Arm picks up cube from 40 cm pedestal in ≤10 seconds, 90% success rate over 10 trials. |
| FR-04 | The robot SHALL autonomously detect and locate cubes at pickup stations using onboard sensors. | Critical | Camera system detects cube position within ±2 cm at 30 cm distance. |
| FR-05 | The robot SHALL read QR codes on cubes to determine the assigned color (Red, Blue, or Green). | Critical | QR code decoded correctly within 3 seconds, 95% success rate under competition lighting. |
| FR-06 | The robot SHALL sort cubes into the correct on-board storage bins based on QR-decoded color. | High | Cubes placed in correct color bin 100% of the time (given correct QR read). |
| FR-07 | The mobile base SHALL execute a pure Y-axis translation for the Red station approach and placement. | Critical | Robot moves forward/backward only, no lateral drift >1 cm over 50 cm travel. |
| FR-08 | The mobile base SHALL execute a pure X-axis strafing motion for the Blue station approach and placement. | Critical | Robot strafes laterally only, no forward/backward drift >1 cm over 50 cm travel. |
| FR-09 | The mobile base SHALL execute a rotate-in-place motion for the Green station approach and placement. | Critical | Robot rotates about its center, translational displacement <2 cm during 90° rotation. |
| FR-10 | The manipulator module SHALL be mechanically and electrically detachable from the base module. | High | Modules can be separated and reconnected within 2 minutes using defined interfaces. |
| FR-11 | Each module SHALL have its own dedicated microcontroller and power distribution. | High | Base and arm each operate independently when disconnected (basic function test). |
| FR-12 | Modules SHALL communicate via a defined digital protocol with heartbeat and safety states. | High | Heartbeat messages exchanged at ≥2 Hz; loss of heartbeat triggers safe stop within 500 ms. |
| FR-13 | The robot SHALL store at least 3 cubes simultaneously during transit. | High | On-board storage securely holds 3 cubes without falling during motion. |
| FR-14 | The robot SHALL deliver cubes to matching colored drop-off zones on the ground. | Critical | Cube ends fully inside the target square at drop-off zone. |
| FR-15 | Encoder and IMU readings SHALL be displayed on a laptop screen during constrained maneuvers. | Medium | Real-time display of wheel encoder counts and IMU heading during Pure X/Y/Rotate. |
| FR-16 | The detected box color SHALL be displayed on a laptop screen after QR reading. | Medium | Color name printed on screen within 1 second of detection. |

---

## 4. Non-Functional Requirements

| ID | Requirement | Value | Verification |
|----|------------|-------|-------------|
| NFR-01 | Total robot mass (including motors, mechanics, electronics, battery) | < 10 kg | Weigh on scale before competition. |
| NFR-02 | Maximum dimensions with arm folded | 500 mm (W) x 500 mm (L) x 700 mm (H) | Measure with calipers/ruler before competition. |
| NFR-03 | Power source | Battery operated (no tethered power) | Visual inspection. |
| NFR-04 | Safety inspection | Pass all 6 Phase 1 safety criteria | Safety inspection checklist. |
| NFR-05 | Emergency stop response time | All actuators halted within 500 ms of ESTOP activation | Timed test with stopwatch/oscilloscope. |
| NFR-06 | Battery runtime | ≥ 30 minutes of active operation | Timed run test. |
| NFR-07 | Wireless control range | ≥ 10 meters line-of-sight | Range test in open area. |
| NFR-08 | Autonomous operation | No human intervention after transition to autonomous mode | Competition rules compliance. |
| NFR-09 | Modularity | Modules integrate via plug-and-play interfaces within 2 minutes | Timed assembly test. |
| NFR-10 | Repeatability | Perform pick-and-place task successfully ≥ 8/10 consecutive runs | Repeatability test protocol. |

---

## 5. Interface Requirements

### 5.1 Mechanical Interface (Base ↔ Arm Module)

| Parameter | Specification |
|-----------|--------------|
| Mounting pattern | 4x M4 bolt holes, 80 mm x 80 mm square pattern |
| Alignment | 2x dowel pins (ø4 mm) for repeatable positioning |
| Mounting surface | Flat aluminum plate, 3 mm thick minimum |
| Attachment method | Thumb screws or quick-release clamps for fast detachment |

### 5.2 Electrical Interface (Base ↔ Arm Module)

| Signal | Connector Pin | Description |
|--------|--------------|-------------|
| CAN_H | 1 | CAN bus high line |
| CAN_L | 2 | CAN bus low line |
| +12V_RAW | 3 | Unregulated battery power (11.1V nominal) |
| GND | 4 | Common ground |
| ESTOP | 5 | Emergency stop signal (active low, open-drain) |
| +5V_SERVO | 6 | Regulated 5V for arm servos |

**Connector:** GX16-6 (6-pin aviation connector), keyed to prevent misalignment.

### 5.3 Communication Protocol (RPi ↔ ESP32 / ESP32 ↔ ESP32)

**UART frame format (RPi to/from ESP32):**

```
| START (0xAA) | MSG_ID (1B) | LENGTH (1B) | PAYLOAD (N bytes) | CRC8 (1B) | END (0x55) |
```

**Key message types:**

| MSG_ID | Name | Direction | Payload Format |
|--------|------|-----------|----------------|
| 0x01 | CMD_VELOCITY | RPi → Base | vx(f32), vy(f32), omega(f32) — 12 bytes |
| 0x02 | ODOMETRY | Base → RPi | x(f32), y(f32), theta(f32), vx(f32), vy(f32) — 20 bytes |
| 0x03 | CMD_ARM_JOINT | RPi → Arm | j1(f32), j2(f32), j3(f32), j4(f32) — 16 bytes |
| 0x04 | CMD_GRIPPER | RPi → Arm | state(u8): 0=open, 1=close — 1 byte |
| 0x05 | ARM_STATUS | Arm → RPi | j1-j5(5xf32), gripper(u8) — 21 bytes |
| 0x06 | MODE_SWITCH | RPi → Both | mode(u8): 0=TELEOP, 1=AUTO, 2=ESTOP — 1 byte |
| 0x07 | HEARTBEAT | Bidirectional | timestamp(u32), status(u8) — 5 bytes |
| 0x08 | SENSOR_DATA | Base → RPi | imu_yaw(f32), enc1-4(i32) — 20 bytes |

**CAN bus (inter-module):** CAN 2.0B standard frame, 500 kbps. Used for heartbeat, ESTOP propagation, and direct module-to-module coordination.

### 5.4 Operator Interface

| Interface | Description |
|-----------|-------------|
| Gamepad | Wireless USB gamepad (e.g., PS4/Xbox controller via Bluetooth to RPi) |
| Left stick | Control vx (forward/backward) and vy (left/right strafe) |
| Right stick | Control omega (rotation) |
| Button A | Switch to autonomous mode |
| Button B | Emergency stop |
| Laptop screen | Display encoder/IMU readings and detected box color |

---

## 6. Constraints

| Parameter | Constraint | Notes |
|-----------|-----------|-------|
| Budget | Flexible (~$340-465 estimated BOM) | See BOM in hardware/mechanical/bom/ |
| Timeline | 15 weeks (Spring 2026 semester) | Aligned with VDI 2206 phases |
| Manufacturing | 3D printing, laser cutting, standard machining | University lab access |
| Component lead time | < 2 weeks | Order critical parts by Week 2 |
| Cube dimensions | 5 cm x 5 cm x 5 cm | Each cube has a QR code |
| Pedestal height | ~40 cm | Arm must reach this height |
| Track lane width | 0.8 m | Two parallel black boundary lines |
| Competition time | 5 minutes per run | Unless instructors announce otherwise |
| Box materials | 3D printing and/or laser cutting | Other methods require instructor approval |

---

## 7. Acceptance Criteria Summary

| Test | Pass Criteria |
|------|--------------|
| Safety inspection | All 6 checklist items passed |
| Manual teleoperation | Robot navigates from start to transition line under gamepad control |
| Autonomous cube pickup | Robot detects, approaches, and picks cube without human intervention |
| QR code reading | Correct color decoded and displayed on laptop screen |
| Color sorting | Cube placed in correct on-board bin |
| Constrained maneuvers | Pure Y, Pure X, Rotate executed with <2 cm drift |
| Drop-off delivery | Cube ends fully inside target zone square |
| Repeatability | ≥ 8/10 successful consecutive runs |
| Weight/dimension check | < 10 kg, < 500x500x700 mm with arm folded |

---

## 8. Traceability Matrix

| Requirement | Competition Rule Source | VDI 2206 Phase |
|-------------|----------------------|----------------|
| FR-01 | "Holonomic motion using omni wheels" | Phase 1 — System Design |
| FR-02 | "Manual wireless teleoperation for track movement" | Phase 2 — Domain Design |
| FR-03, FR-04 | "Grasping predefined competition objects" | Phase 2 — Domain Design |
| FR-05 | "QR code that encodes the target box color" | Phase 2 — Domain Design |
| FR-06 | "Color-based sorting into dedicated on-board bins" | Phase 2 — Domain Design |
| FR-07 | "Station A - RED (Pure Y Move)" | Phase 4 — Implementation |
| FR-08 | "Station B - BLUE (Pure X Move)" | Phase 4 — Implementation |
| FR-09 | "Station C - GREEN (Rotation Station)" | Phase 4 — Implementation |
| FR-10, FR-11 | "Each module must have its own controller" | Phase 1 — System Design |
| FR-12 | "Defined communication protocol including heartbeat" | Phase 1 — System Design |
| NFR-01 | "Less than 10kg" | Phase 0 — Requirements |
| NFR-02 | "50 cm x 50 cm x 70 cm" | Phase 0 — Requirements |
| NFR-04 | "Mandatory safety inspection" | Phase 4 — Verification |
