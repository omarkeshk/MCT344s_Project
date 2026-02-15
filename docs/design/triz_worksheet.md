# TRIZ Ideation Worksheet — Modular Omni-Wheel Mobile Manipulator

**VDI 2206 Phase:** Phase 1 — Concept Generation
**Method:** TRIZ (Theory of Inventive Problem Solving)

---

## 1. System Contradictions and Inventive Principles

### Contradiction 1: Omnidirectional Motion Capability vs. Weight and Cost

**Improving parameter:** Versatility / adaptability (holonomic motion in any direction)
**Worsening parameter:** Weight of moving object, complexity, cost

**TRIZ Contradiction Matrix suggests:** Principles #1, #15, #35, #28

| Principle | Application | Resulting Concept |
|-----------|-------------|-------------------|
| #1 — Segmentation | Divide the drive system into independent wheel modules, each providing a directional component | 4 independent mecanum wheel units that combine for holonomic motion |
| #15 — Dynamization | Use passive rollers on wheels that adapt direction based on wheel spin combination | Mecanum rollers provide omnidirectional capability without additional actuators |
| #35 — Parameter Change | Vary the number of wheels (3 vs 4) and arrangement angle to balance capability vs. cost | 4-wheel rectangular arrangement gives best stability-to-cost ratio |
| #28 — Mechanics Substitution | Replace complex steering mechanisms with wheel roller geometry | Mecanum/omni wheels eliminate the need for steering actuators entirely |

**Selected concept:** 4x Mecanum wheels in rectangular layout. Passive rollers eliminate steering actuators while providing full holonomic motion. The rectangular frame maximizes stability for arm mounting.

---

### Contradiction 2: Arm Reach (40 cm pedestal height) vs. Stability and Weight

**Improving parameter:** Length of moving object (arm reach to 40 cm height)
**Worsening parameter:** Stability of object composition, weight

**TRIZ Contradiction Matrix suggests:** Principles #8, #17, #3, #40

| Principle | Application | Resulting Concept |
|-----------|-------------|-------------------|
| #8 — Counterweight | Place the battery (heaviest single component) on the opposite side of the arm | Battery positioned at rear of base, arm at front — balanced CG |
| #17 — Another Dimension | Use the base rotation (yaw) to reduce required linear reach of the arm | Base rotation joint provides 360° yaw, reducing horizontal reach requirement |
| #3 — Local Quality | Use high-torque servos only at base joints (where load is highest) and lighter servos at distal joints | DS3218 (20 kg-cm) at shoulder, MG90S at wrist — weight proportional to need |
| #40 — Composite Materials | Use lightweight materials for arm links (3D-printed PLA/PETG or aluminum) | Arm links 3D-printed to minimize weight while maintaining stiffness |

**Selected concept:** 4-DOF articulated arm with graded servo sizing (heavy-to-light from base to tip). Battery counterweight positioning. Base yaw joint lets the arm use the robot's rotation to face targets, reducing required horizontal reach.

---

### Contradiction 3: Autonomous Precision (QR detection, navigation) vs. Embedded Processing Power

**Improving parameter:** Accuracy of detection, speed of autonomous operation
**Worsening parameter:** Complexity of device, energy consumption, cost

**TRIZ Contradiction Matrix suggests:** Principles #5, #24, #10, #3

| Principle | Application | Resulting Concept |
|-----------|-------------|-------------------|
| #24 — Intermediary | Introduce a higher-powered processor (RPi) as intermediary between sensors and actuators | Raspberry Pi handles vision/planning, ESP32s handle real-time control |
| #5 — Merging | Combine QR code reading and color detection into a single camera pipeline | One camera serves both QR decoding (pyzbar) and color verification (HSV) |
| #10 — Preliminary Action | Pre-position camera angle and exposure before arriving at pickup station | Camera calibrated and auto-exposed before approaching cube, reducing detection time |
| #3 — Local Quality | Assign each processor to what it does best — MCU for real-time, SBC for vision | ESP32: deterministic motor control at 1 kHz. RPi: OpenCV at 30 fps. |

**Selected concept:** Hybrid architecture — two ESP32-S3 MCUs for real-time motor/servo control, one Raspberry Pi 4B for perception and mission planning. Single camera pipeline handles both QR and color detection.

---

### Contradiction 4: Modularity (plug-and-play) vs. System Integration Speed

**Improving parameter:** Ease of assembly / disassembly
**Worsening parameter:** Reliability of connections, signal integrity, integration complexity

**TRIZ Contradiction Matrix suggests:** Principles #1, #25, #20, #15

| Principle | Application | Resulting Concept |
|-----------|-------------|-------------------|
| #1 — Segmentation | Clearly separate into two self-contained modules with defined boundaries | Base module and arm module, each with own MCU, power regulation, and firmware |
| #25 — Self-Service | Modules auto-detect each other via CAN heartbeat when connected | Plug in the connector → CAN heartbeat detected → modules coordinate automatically |
| #20 — Continuity of Useful Action | Use a single multi-pin connector for all signals (power + data + safety) | GX16-6 aviation connector carries power, CAN, and ESTOP in one plug |
| #15 — Dynamization | Allow modules to operate in degraded mode when partner is absent | Base drives independently without arm; arm tests servos independently without base |

**Selected concept:** Single aviation connector interface carrying all signals. CAN bus heartbeat for auto-detection. Each module operates independently for testing, integrates seamlessly when connected.

---

### Contradiction 5: Navigation Accuracy (odometry) vs. Sensor Drift Over Time

**Improving parameter:** Measurement accuracy (position estimation)
**Worsening parameter:** Reliability / drift over extended operation

**TRIZ Contradiction Matrix suggests:** Principles #5, #32, #23, #28

| Principle | Application | Resulting Concept |
|-----------|-------------|-------------------|
| #5 — Merging | Fuse multiple sensor sources (wheel encoders + IMU) to compensate individual weaknesses | Complementary filter: encoders provide translation, IMU provides heading correction |
| #32 — Color Change | Use visual markers (lane lines, station markers) as absolute reference points | Camera detects lane boundaries for lateral position correction |
| #23 — Feedback | Close the loop — use sensor feedback to correct accumulated errors | Odometry error detected via IMU heading drift → correction applied to wheel estimates |
| #28 — Mechanics Substitution | Replace pure dead-reckoning with sensor fusion algorithm | Kalman filter or complementary filter replaces simple encoder integration |

**Selected concept:** Encoder + IMU complementary filter for real-time odometry. IMU heading prevents yaw drift during strafing. Optional visual landmark correction for long-range accuracy.

---

## 2. Concept Alternatives Table

| Function | Concept A | Concept B | Concept C | Concept D |
|----------|-----------|-----------|-----------|-----------|
| **Locomotion** | 4x Mecanum wheels (rectangular) | 3x Omni wheels (120°) | 4x Omni wheels (X-drive) | Swerve drive |
| **Arm configuration** | 4-DOF serial articulated | SCARA-style (horizontal reach) | 3-DOF + linear rail | Parallel linkage (delta) |
| **Gripper** | Servo-driven 2-finger parallel | Vacuum suction cup | Compliant soft gripper | Magnetic (if cube has metal) |
| **Vision system** | RPi + USB/CSI camera | ESP32-CAM standalone | Dual camera stereo | Depth camera (RealSense) |
| **QR detection** | pyzbar on RPi (Python) | ZBar on ESP32 (C) | Custom CNN detector | ArUco markers (OpenCV) |
| **Inter-module comm** | CAN bus (TWAI) | UART daisy-chain | WiFi (ESP-NOW) | RS-485 |
| **Power source** | 3S LiPo 11.1V 5000mAh | 4S LiPo 14.8V 3000mAh | NiMH pack 12V | USB power bank (5V only) |
| **Frame material** | Aluminum extrusion + 3D print | Full 3D print (PETG) | Laser-cut acrylic | Laser-cut MDF + aluminum |
| **Odometry fusion** | Complementary filter | Extended Kalman Filter | Madgwick filter | Pure encoder (no fusion) |
| **Supervisory control** | Raspberry Pi 4B (Python) | Laptop via WiFi (Python) | Jetson Nano | ESP32 only (no SBC) |

---

## 3. Concept Selection Rationale

The selected concepts (Concept A in most categories) were chosen based on the Pugh matrix evaluation in the separate `concept_selection.md` document. Key reasons:

- **4x Mecanum:** Most stable platform, direct mapping to competition Pure X/Y/Rotate maneuvers, well-documented kinematics
- **4-DOF serial arm:** Simplest IK solution for reaching 40 cm height, sufficient DOF, widely available servos
- **Servo 2-finger gripper:** Simple, reliable for 5 cm cubes, no compressed air or vacuum pump needed
- **RPi + camera:** Best QR detection reliability, fast Python development, sufficient compute for OpenCV
- **CAN bus:** Robust against noise, native ESP32-S3 support, industry-standard for modular systems
- **3S LiPo:** Voltage matches 12V motors directly, good energy density, common and affordable
- **Complementary filter:** Simple to implement, sufficient accuracy for competition distances (~10 m total)
