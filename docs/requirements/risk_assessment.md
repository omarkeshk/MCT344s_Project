# Risk Assessment — Modular Omni-Wheel Mobile Manipulator

**VDI 2206 Phase:** Phase 0 — Problem Definition
**Method:** FMEA-style risk identification with likelihood/impact scoring

---

## Risk Scoring

- **Likelihood:** Low (1), Medium (2), High (3)
- **Impact:** Low (1), Medium (2), High (3), Critical (4)
- **Risk Score:** Likelihood x Impact (higher = more urgent)

---

## Risk Register

| ID | Risk Description | Likelihood | Impact | Score | Mitigation Strategy | Owner |
|----|-----------------|-----------|--------|-------|---------------------|-------|
| R01 | Motor torque insufficient for mecanum wheels on competition floor (carpet/concrete transition) | Medium (2) | High (3) | 6 | Test motors with loaded robot on actual floor surface by Week 5. Select motor with 2x safety margin on torque. Have backup higher-torque motor variant identified. | Actuation Lead |
| R02 | Arm cannot reliably reach cubes on 40 cm pedestal | Medium (2) | Critical (4) | 8 | Complete kinematic analysis in CAD before manufacturing. Verify reach envelope with 5 cm margin. Test with prototype arm by Week 7. | Mechanical Lead |
| R03 | QR code detection fails under competition lighting (glare, shadows, uneven illumination) | High (3) | High (3) | 9 | Test with multiple lighting conditions (fluorescent, LED, natural). Implement backup color-only detection via HSV. Use camera auto-exposure. Add LED ring light as contingency. | Software Lead |
| R04 | CAN bus communication drops or corrupts between modules | Low (1) | Medium (2) | 2 | Use proper CAN transceivers (SN65HVD230) with 120Ω termination resistors. Implement heartbeat monitoring with 500 ms timeout. Test with cable lengths up to 1 m. | Electronics Lead |
| R05 | Robot exceeds 10 kg weight limit | Low (1) | Critical (4) | 4 | Track component weights in BOM from Week 1. Weigh every component on receipt. Current estimate ~4 kg leaves 6 kg margin. Weigh assembled robot weekly. | Project Manager |
| R06 | Robot exceeds 500x500x700 mm dimension limit (arm folded) | Low (1) | Critical (4) | 4 | Define arm folded position in CAD. Measure assembled robot with ruler/calipers. Design arm fold mechanism that fits within 700 mm height. | Mechanical Lead |
| R07 | Battery runtime insufficient for competition | Low (1) | Medium (2) | 2 | Power budget analysis (est. 3.5A avg, 5Ah battery = 1.4 hr). Monitor actual current draw during testing. 5-minute competition run is well within margin. | Actuation Lead |
| R08 | Critical parts delayed in shipping | Medium (2) | High (3) | 6 | Order motors, wheels, ESP32s, RPi, servos, battery by Week 2. Identify backup suppliers. Have at least one spare ESP32 and motor. | Project Manager |
| R09 | Mecanum wheel slippage causes inaccurate Pure X/Y motion | High (3) | Medium (2) | 6 | IMU heading correction during strafing. Test on actual competition floor surface. Calibrate wheel odometry scaling factors. Increase wheel normal force if needed. | Controls Lead |
| R10 | Odometry drift accumulates over track length (~10 m) | High (3) | Medium (2) | 6 | Fuse wheel encoders + IMU heading. Calibrate encoder ticks-per-meter carefully. Use IMU to correct yaw drift. Consider visual landmark corrections. | Controls Lead |
| R11 | Gripper fails to grasp cube (drops, misaligns) | Medium (2) | High (3) | 6 | Design gripper opening wider than cube (>6 cm). Add rubber pads for grip. Test grasp repeatability (10+ trials). Implement grasp detection (current sensing or timeout). | Mechanical Lead |
| R12 | ESP32-S3 runs out of MCPWM/PCNT resources for 4 motors | Low (1) | High (3) | 3 | Verified: ESP32-S3 has 2 MCPWM groups (6 operators) and 4 PCNT units — exactly sufficient. If needed, use LEDC for simpler PWM on 2 motors. | Embedded Lead |
| R13 | Wireless teleoperation latency too high (>200 ms) | Low (1) | Medium (2) | 2 | Use direct Bluetooth gamepad to RPi (low latency). Avoid WiFi congestion. Test in competition venue if possible. | Software Lead |
| R14 | Safety inspection failure (wiring, sharp edges, fuse) | Medium (2) | Critical (4) | 8 | Follow safety checklist from Week 7 onward. Use proper wire harnessing, strain relief. Deburr all metal edges. Install main fuse and accessible power switch. Conduct internal safety review at Week 11. | Electronics Lead |
| R15 | Team member unavailable for critical deadline | Medium (2) | High (3) | 6 | Cross-train on adjacent roles. Document all interfaces and designs. Maintain responsibility sheet with backup assignees. Weekly integration checkpoints. | Project Manager |
| R16 | 3D printed parts break under load | Medium (2) | Medium (2) | 4 | Use PETG instead of PLA for structural parts. Design with generous wall thickness (3+ mm). Print with 40%+ infill for load-bearing parts. Test before competition. | Mechanical Lead |
| R17 | Servo overheating during extended arm operation | Medium (2) | Medium (2) | 4 | Limit continuous servo duty. Return arm to rest position between tasks. Use metal-gear servos (MG series). Monitor servo temperature during testing. | Actuation Lead |
| R18 | Camera field of view insufficient to detect cubes at pickup stations | Low (1) | High (3) | 3 | Select wide-angle camera or use RPi Camera v3 (75° FoV). Mount camera at appropriate height/angle for 30-50 cm working distance. Test detection range early. | Software Lead |

---

## Risk Matrix Visualization

```
                        Impact
                 Low(1)  Med(2)  High(3)  Critical(4)
Likelihood
  High(3)    |        | R09,R10|  R03   |            |
  Medium(2)  |        | R16,R17|R01,R08 |  R02,R14   |
             |        |        |R11,R15 |            |
  Low(1)     |        |R04,R07 |R12,R18 |  R05,R06   |
             |        |  R13   |        |            |
```

---

## Top Priority Risks (Score ≥ 6)

| Priority | Risk | Score | Action Due |
|----------|------|-------|-----------|
| 1 | R03 — QR detection under lighting | 9 | Test by Week 5, backup pipeline by Week 9 |
| 2 | R02 — Arm reach to 40cm | 8 | CAD verification by Week 3, prototype by Week 7 |
| 3 | R14 — Safety inspection failure | 8 | Checklist compliance from Week 7 |
| 4 | R01 — Motor torque on floor | 6 | Motor test by Week 5 |
| 5 | R08 — Parts shipping delay | 6 | Order by Week 2 |
| 6 | R09 — Mecanum slip during strafing | 6 | IMU correction by Week 9, floor test by Week 11 |
| 7 | R10 — Odometry drift | 6 | Sensor fusion by Week 9 |
| 8 | R11 — Gripper reliability | 6 | Repeatability test by Week 9 |
| 9 | R15 — Team member unavailability | 6 | Cross-training from Week 1 |

---

## Risk Review Schedule

| Week | Review Action |
|------|--------------|
| Week 3 | Review R02 (arm reach CAD), R08 (parts ordered?) |
| Week 5 | Review R01 (motor test), R03 (QR lighting test) |
| Week 7 | Review R02 (arm prototype), R14 (safety checklist start) |
| Week 9 | Review R03 (backup pipeline), R09/R10 (odometry), R11 (gripper) |
| Week 11 | Review R14 (internal safety audit), R09 (competition floor test) |
| Week 13 | Final risk review — all items must be mitigated or accepted |
