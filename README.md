# MCT344s — Modular Omni-Wheel Mobile Manipulator

Spring 2026 Mechatronics Omni-Challenge | Ain Shams University

## Project Overview

A modular mobile manipulator robot composed of an omni-wheel mobile base and a robotic arm with a gripper. The system is developed as two plug-and-play modules that integrate through standardized mechanical, electrical, and communication interfaces.

**Competition task:** Navigate a marked track via manual teleoperation, then autonomously pick up QR-coded cubes from pedestals, sort them by color (Red/Blue/Green), and deliver them to matching drop-off zones using constrained maneuvers (Pure Y, Pure X, Rotate-in-place).

## System Architecture

| Layer | Hardware | Role |
|-------|----------|------|
| Supervisory | Raspberry Pi 4B | Perception (QR/color detection), mission planning, teleoperation |
| Base Module | ESP32-S3 | 4x Mecanum motor PID control, odometry, IMU fusion |
| Arm Module | ESP32-S3 | 4-DOF arm servo control, inverse kinematics, gripper |

**Inter-module communication:** CAN bus (TWAI) between ESP32s, UART over USB to Raspberry Pi.

## Repository Structure

```
docs/           — Requirements, design documents, reports
hardware/       — CAD files, schematics, PCB layouts, BOM
firmware/       — ESP-IDF projects (base_controller, arm_controller)
software/       — Python supervisory code (master_node)
tests/          — Unit, integration, and system tests
tools/          — Build scripts, calibration utilities
```

## Key Specifications

| Parameter | Value |
|-----------|-------|
| Max weight | < 10 kg (est. ~4 kg) |
| Max dimensions (arm folded) | 500 x 500 x 700 mm |
| Drive | 4x Mecanum wheels (holonomic) |
| Arm | 4-DOF + gripper (reaches 40 cm pedestal) |
| Power | 3S LiPo 11.1V 5000mAh |
| Vision | RPi Camera + OpenCV + pyzbar |

## Getting Started

### Prerequisites

- ESP-IDF v5.x+ (for firmware development)
- Python 3.10+ (for supervisory software)
- OpenCV, pyzbar (for perception)

### Building Firmware

```bash
cd firmware/base_controller
idf.py set-target esp32s3
idf.py build
idf.py flash
```

### Running Supervisory Software

```bash
cd software/master_node
pip install -r requirements.txt
python src/main.py
```

## Team

| Role | Name |
|------|------|
| Project Manager & Systems Engineer | TBD |
| Mechanical Lead | TBD |
| Actuation & Power Lead | TBD |
| Electronics Lead | TBD |
| Embedded/Real-Time Lead | TBD |
| Software/Perception Lead | TBD |
| Controls & Estimation Lead | TBD |
| Documentation & QA Lead | TBD |

## License

TBD
