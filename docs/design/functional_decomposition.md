# Functional Decomposition — Modular Omni-Wheel Mobile Manipulator

**VDI 2206 Phase:** Phase 1 — System-Level Design

---

## 1. Overall System Function

**Input:** Operator commands (gamepad), environment information (camera, sensors), electrical energy (battery)
**Output:** Sorted cubes delivered to correct drop-off zones, telemetry data on display
**Overall Function:** Navigate competition track, identify and pick up cubes, sort by color, and deliver to matching zones.

---

## 2. Level 1 — Primary Functions

| ID | Function | Input | Output |
|----|----------|-------|--------|
| F1 | Provide holonomic mobile locomotion | Velocity commands, electrical energy | Robot translation/rotation |
| F2 | Manipulate objects (pick and place) | Joint commands, electrical energy | Cube grasped/released at target position |
| F3 | Perceive environment | Light (camera), reflected signals (sensors) | Cube location, QR data, obstacle map |
| F4 | Plan and execute mission | Perception data, odometry, mode state | Sequenced actions (navigate, pick, sort, deliver) |
| F5 | Receive and process operator input | Gamepad signals (wireless) | Velocity commands, mode switches |
| F6 | Supply and distribute electrical power | Chemical energy (battery) | Regulated voltages (12V, 5V, 3.3V) |
| F7 | Communicate between modules | Digital signals | Coordinated multi-module behavior |

---

## 3. Level 2 — Sub-Functions

### F1: Provide Holonomic Mobile Locomotion

| ID | Sub-Function | Input | Output |
|----|-------------|-------|--------|
| F1.1 | Convert electrical energy to rotational motion | PWM signals, 12V power | Motor shaft rotation |
| F1.2 | Transform rotation to omnidirectional translation | Shaft rotation via mecanum wheels | Robot X/Y/θ motion |
| F1.3 | Measure wheel rotation | Magnetic flux changes (encoder) | Pulse counts (quadrature) |
| F1.4 | Measure orientation and angular rate | Inertial forces (IMU) | Heading angle, angular velocity |
| F1.5 | Compute position estimate (odometry) | Encoder counts, IMU data | Estimated pose (x, y, θ) |
| F1.6 | Regulate wheel speeds | Desired vs. actual speed | Corrected PWM duty cycle |
| F1.7 | Compute wheel velocities from desired motion | vx, vy, ω (body frame) | ω1, ω2, ω3, ω4 (wheel speeds) |

### F2: Manipulate Objects

| ID | Sub-Function | Input | Output |
|----|-------------|-------|--------|
| F2.1 | Rotate arm base to face target | Target angle command | Arm yaw rotation |
| F2.2 | Position arm end-effector in workspace | Target (x, y, z) in arm frame | Shoulder and elbow angles |
| F2.3 | Orient gripper for grasp | Desired gripper angle | Wrist pitch rotation |
| F2.4 | Grasp/release object | Open/close command | Cube clamped/released |
| F2.5 | Solve inverse kinematics | Target Cartesian position | Joint angle setpoints |
| F2.6 | Interpolate trajectory | Start/end joint angles | Smooth motion profile |
| F2.7 | Detect successful grasp | Gripper feedback (position/current) | Grasp confirmed/failed signal |

### F3: Perceive Environment

| ID | Sub-Function | Input | Output |
|----|-------------|-------|--------|
| F3.1 | Capture camera images | Light from scene | Raw image frames (640x480) |
| F3.2 | Detect QR codes in image | Camera frame | QR bounding box, decoded string |
| F3.3 | Classify cube color | QR data or HSV image region | Color ID (Red/Blue/Green) |
| F3.4 | Estimate cube position relative to robot | QR/contour position + size in image | Cube distance and bearing |
| F3.5 | Detect obstacles | Range sensor data (optional) | Obstacle distance and direction |
| F3.6 | Undistort and calibrate camera | Raw image, calibration matrix | Corrected image |

### F4: Plan and Execute Mission

| ID | Sub-Function | Input | Output |
|----|-------------|-------|--------|
| F4.1 | Manage operating mode | Mode switch command | Active mode (TELEOP/AUTO/ESTOP) |
| F4.2 | Sequence autonomous tasks | Perception data, current state | Next action (approach/pick/navigate/place) |
| F4.3 | Plan path to target location | Current pose, target pose | Waypoint sequence |
| F4.4 | Track reference trajectory | Waypoints, current pose | Velocity commands (vx, vy, ω) |
| F4.5 | Decide cube-to-zone mapping | Decoded QR color | Target drop-off zone coordinates |
| F4.6 | Handle errors and retries | Failed action feedback | Recovery action or abort |

### F5: Receive and Process Operator Input

| ID | Sub-Function | Input | Output |
|----|-------------|-------|--------|
| F5.1 | Read gamepad axes and buttons | Wireless gamepad USB/BT | Axis values (-1 to +1), button states |
| F5.2 | Map gamepad to velocity commands | Axis values | vx, vy, ω scaled to robot limits |
| F5.3 | Detect mode switch requests | Button press events | Mode transition signal |
| F5.4 | Apply deadzone and scaling | Raw axis values | Cleaned velocity commands |

### F6: Supply and Distribute Electrical Power

| ID | Sub-Function | Input | Output |
|----|-------------|-------|--------|
| F6.1 | Store electrical energy | Charged LiPo battery | 11.1V DC (nominal) |
| F6.2 | Protect against overcurrent | All load current | Fuse interrupts on fault |
| F6.3 | Enable/disable power | Switch actuation | System power on/off |
| F6.4 | Regulate to 5V | 11.1V input | 5V regulated output (for RPi, servos) |
| F6.5 | Regulate to 3.3V | 5V input (on-board ESP32 regulator) | 3.3V for MCU logic |
| F6.6 | Monitor battery voltage | Battery voltage (ADC) | Low-battery warning signal |
| F6.7 | Emergency power cutoff | ESTOP signal | All power interrupted |

### F7: Communicate Between Modules

| ID | Sub-Function | Input | Output |
|----|-------------|-------|--------|
| F7.1 | Serialize messages into frames | Structured data (commands, telemetry) | Byte stream with framing |
| F7.2 | Transmit over UART (RPi ↔ ESP32) | Byte stream | Physical UART signal |
| F7.3 | Transmit over CAN (ESP32 ↔ ESP32) | CAN frame data | Differential CAN bus signal |
| F7.4 | Parse received messages | Incoming byte stream/CAN frame | Structured data |
| F7.5 | Validate message integrity | Received CRC | Accept/reject decision |
| F7.6 | Generate heartbeat | Timer tick | Periodic heartbeat message |
| F7.7 | Detect communication loss | Missing heartbeats | Timeout/safety trigger |

---

## 4. Function Structure Diagram (Flow Description)

### Energy Flow
```
Battery (chemical energy)
  → F6.1 Store energy (11.1V DC)
    → F6.4 Regulate to 5V
      → F5 Operator interface (RPi)
      → F3 Perception (camera, RPi)
      → F2 Arm servos
    → F6.5 Regulate to 3.3V
      → F7 Communication (ESP32 logic)
    → F1.1 Motor drive (12V to motors)
      → F1.2 Mecanum wheel motion (mechanical energy)
```

### Signal Flow
```
F5.1 Gamepad input
  → F5.2 Map to velocity
    → F7.1 Serialize command
      → F7.2 UART transmit to base ESP32
        → F7.4 Parse command
          → F1.7 Inverse kinematics (vx,vy,ω → wheel speeds)
            → F1.6 PID control (regulate each wheel)
              → F1.1 Motor PWM output

F1.3 Encoder pulses + F1.4 IMU data
  → F1.5 Odometry computation
    → F7.1 Serialize odometry
      → F7.2 UART transmit to RPi

F3.1 Camera capture
  → F3.6 Undistort
    → F3.2 QR detection → F3.3 Color classify
    → F3.4 Cube position estimate
      → F4.2 Mission sequencer
        → F4.3 Path planning
          → F4.4 Trajectory tracking
            → Velocity commands → F1 (locomotion loop)
        → F2.5 Arm IK → F2.1-F2.4 (arm motion)
```

### Material Flow
```
Cube (on 40cm pedestal)
  → F2.4 Gripper grasps cube
    → F1.2 Base transports cube (on-board)
      → F2.4 Gripper releases cube at drop-off zone
```

---

## 5. Module Boundary Mapping

| Function | Module Assignment |
|----------|------------------|
| F1 (Locomotion) | Base Module (ESP32-S3 #1) |
| F2 (Manipulation) | Arm Module (ESP32-S3 #2) |
| F3 (Perception) | Supervisory (Raspberry Pi) |
| F4 (Mission Planning) | Supervisory (Raspberry Pi) |
| F5 (Operator Input) | Supervisory (Raspberry Pi) |
| F6 (Power) | Distributed — base provides power, arm has local regulation |
| F7 (Communication) | Distributed — all modules participate |
