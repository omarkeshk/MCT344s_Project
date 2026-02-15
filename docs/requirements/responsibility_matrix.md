# Responsibility Matrix — Team Role Assignment

**Updated:** Week 0 (Initial Setup)
**Review frequency:** Weekly

---

## Team Roles

| Role | Name / ID | Primary Responsibilities |
|------|-----------|------------------------|
| Project Manager & Systems Engineer | TBD | Requirements, schedules, integration readiness reviews, risk management |
| Mechanical Lead | TBD | Base/arm CAD, manufacturing drawings, assembly, structural design |
| Actuation & Power Lead | TBD | Motor sizing, transmission selection, battery sizing, fusing, power board |
| Electronics Lead | TBD | Schematics, PCB layout, wiring, connectors, protection circuits |
| Embedded/Real-Time Lead | TBD | MCU firmware, motor control loops, safety state machine, CAN/UART |
| Software/Perception Lead | TBD | Camera pipeline, QR detection, color classification, telemetry display |
| Controls & Estimation Lead | TBD | Kinematics, odometry fusion, trajectory tracking, autonomous behaviors |
| Documentation & QA Lead | TBD | VDI 2206 deliverables, test plans, datasheet, revision control |

---

## Weekly Responsibility Tracker

### Week 1

| Student Name / ID | Role | Owned Tasks | Interfaces / Dependencies | Evidence (links, files, commits) | Hours / Week (est.) | TA / Lead Sign-off |
|-------------------|------|-------------|--------------------------|--------------------------------|---------------------|-------------------|
| | PM | Set up repo, define roles, create schedule | All team members | Git repo URL, responsibility sheet | | |
| | Mech | Survey base frame options, initial sketches | Actuation Lead (motor dimensions) | Sketches in hardware/mechanical/ | | |
| | Actuation | Research motor options, initial torque calcs | Mech Lead (wheel size), Elec Lead (driver specs) | Spreadsheet in hardware/mechanical/bom/ | | |
| | Electronics | Survey ESP32-S3 pinout, plan wiring | Embedded Lead (GPIO allocation) | Pinout diagram draft | | |
| | Embedded | Set up ESP-IDF environment, blink test | Electronics Lead (board selection) | firmware/ commit with build passing | | |
| | Software | Set up Python environment, test pyzbar | Controls Lead (serial protocol) | software/ commit with QR demo | | |
| | Controls | Draft kinematic equations, review literature | Embedded Lead (encoder specs) | Notes in docs/design/ | | |
| | Doc/QA | Set up docs structure, begin requirements | PM (scope), All (inputs) | docs/ structure committed | | |

### Week 2

| Student Name / ID | Role | Owned Tasks | Interfaces / Dependencies | Evidence | Hours | Sign-off |
|-------------------|------|-------------|--------------------------|----------|-------|----------|
| | | | | | | |

*(Copy this table template for each subsequent week)*

---

## Interface Ownership

| Interface | Owner | Collaborator | Document Location |
|-----------|-------|-------------|-------------------|
| Mechanical base ↔ arm mount | Mechanical Lead | Actuation Lead | docs/design/system_architecture.md §4.1 |
| Electrical connector (GX16-6) | Electronics Lead | Actuation Lead | docs/design/system_architecture.md §4.2 |
| UART protocol (RPi ↔ ESP32) | Embedded Lead | Software Lead | docs/design/system_architecture.md §3 |
| CAN protocol (Base ↔ Arm) | Embedded Lead | Controls Lead | docs/design/system_architecture.md §3.4 |
| Motor driver wiring | Actuation Lead | Electronics Lead | hardware/electrical/wiring_diagrams/ |
| Servo wiring | Actuation Lead | Electronics Lead | hardware/electrical/wiring_diagrams/ |
| Camera mounting | Mechanical Lead | Software Lead | hardware/mechanical/cad/ |
| Power distribution | Actuation Lead | Electronics Lead | hardware/electrical/schematics/power_distribution/ |

---

## Decision Log

| Date | Decision | Options Considered | Rationale | Decided By |
|------|----------|--------------------|-----------|-----------|
| Week 0 | 4x Mecanum wheel configuration | 3-omni, 4-mecanum, 4-omni X-drive | Stability, direct Pure X/Y mapping, rectangular frame | Team (see concept_selection.md) |
| Week 0 | ESP32-S3 + RPi 4B hybrid architecture | ESP32-only, ESP32+laptop, ESP32+RPi | QR reliability, self-contained, real-time + vision split | Team (see concept_selection.md) |
| Week 0 | CAN bus inter-module communication | UART, CAN, WiFi/ESP-NOW, RS-485 | Noise immunity, native TWAI support, modularity | Team (see concept_selection.md) |
| Week 0 | 4-DOF serial articulated arm | SCARA, 3-DOF+rail, parallel linkage | Simple IK, sufficient reach, low cost | Team (see concept_selection.md) |
| Week 0 | 3S LiPo 11.1V 5000mAh battery | 4S LiPo, NiMH, USB power bank | Voltage matches motors, ample runtime | Team (see concept_selection.md) |

---

## Notes

- This sheet must be updated **weekly** and reviewed by the TA
- Each student must have clearly defined tasks with measurable deliverables
- Evidence must include links to specific files, commits, or documents
- The decision log captures all major design choices for traceability
