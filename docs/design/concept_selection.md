# Concept Selection — Pugh Matrix Analysis

**VDI 2206 Phase:** Phase 1 — Concept Selection
**Method:** Pugh Matrix (weighted scoring against baseline)

---

## 1. Locomotion Concept Selection

**Baseline (S):** Concept B — 3x Omni Wheels (120° arrangement)

| Criteria | Weight | Concept A: 4 Mecanum | Concept B: 3 Omni (Baseline) | Concept C: 4 Omni X-Drive |
|----------|--------|---------------------|------------------------------|---------------------------|
| Holonomic motion capability | 0.20 | + | S | + |
| Platform stability (with arm) | 0.20 | + | S | + |
| Pure X/Y/Rotate precision | 0.15 | + | S | S |
| Ease of kinematic modeling | 0.10 | + | S | S |
| Weight | 0.10 | - | S | - |
| Cost | 0.10 | - | S | - |
| Ease of frame design | 0.10 | + | S | S |
| Traction on smooth floor | 0.05 | S | S | S |
| **Weighted Score** | | **+0.35** | **0 (baseline)** | **+0.10** |

**Selected: Concept A — 4x Mecanum Wheels**

Rationale: Superior stability for arm mounting (rectangular base), best mapping to competition-required Pure X, Pure Y, and Rotate maneuvers. The weight and cost penalty (one extra motor + wheel) is acceptable given the stability and kinematic advantages.

---

## 2. Arm Configuration Selection

**Baseline (S):** Concept B — SCARA-style arm

| Criteria | Weight | Concept A: 4-DOF Serial | Concept B: SCARA (Baseline) | Concept C: 3-DOF + Rail |
|----------|--------|------------------------|----------------------------|------------------------|
| Vertical reach (40 cm pedestal) | 0.25 | + | S | + |
| IK simplicity | 0.15 | + | S | - |
| Weight | 0.15 | S | S | - |
| Cost | 0.15 | + | S | - |
| Gripper orientation flexibility | 0.10 | + | S | S |
| Ease of 3D printing | 0.10 | + | S | - |
| Stiffness under load | 0.10 | S | S | + |
| **Weighted Score** | | **+0.55** | **0 (baseline)** | **-0.25** |

**Selected: Concept A — 4-DOF Serial Articulated Arm**

Rationale: Best vertical reach for pedestals, simplest inverse kinematics (2-link planar + base yaw + wrist pitch), cheapest servo configuration, easiest to 3D print.

---

## 3. Computing Architecture Selection

**Baseline (S):** Concept D — ESP32-only (no SBC)

| Criteria | Weight | Concept A: ESP32 + RPi | Concept B: ESP32 + Laptop WiFi | Concept D: ESP32 Only (Baseline) |
|----------|--------|----------------------|-------------------------------|--------------------------------|
| QR detection reliability | 0.25 | + | + | S |
| Development speed | 0.20 | + | + | S |
| Latency (camera to action) | 0.15 | + | - | S |
| System self-containment | 0.15 | + | - | S |
| Cost | 0.10 | - | S | S |
| Power consumption | 0.10 | - | S | S |
| Complexity | 0.05 | - | - | S |
| **Weighted Score** | | **+0.30** | **+0.05** | **0 (baseline)** |

**Selected: Concept A — ESP32-S3 x2 + Raspberry Pi 4B**

Rationale: QR detection on RPi is far more reliable than ESP32-CAM. Self-contained (no laptop dependency during competition). Python development on RPi enables rapid iteration on perception and planning code. Cost increase (~$60) is justified by massive reliability improvement.

---

## 4. Communication Protocol Selection

**Baseline (S):** Concept B — UART Daisy-Chain

| Criteria | Weight | Concept A: CAN Bus | Concept B: UART (Baseline) | Concept C: WiFi ESP-NOW |
|----------|--------|-------------------|---------------------------|------------------------|
| Noise immunity | 0.25 | + | S | - |
| Multi-node support | 0.20 | + | S | + |
| Plug-and-play detection | 0.15 | + | S | + |
| Bandwidth | 0.15 | + | S | S |
| Wiring simplicity | 0.10 | S | S | + |
| ESP32 native support | 0.10 | + | S | + |
| Reliability | 0.05 | + | S | - |
| **Weighted Score** | | **+0.65** | **0 (baseline)** | **+0.15** |

**Selected: Concept A — CAN Bus (TWAI)**

Rationale: Best noise immunity (differential signaling), native ESP32-S3 TWAI support, industry-standard for modular robotic systems, supports heartbeat-based auto-detection. Requires CAN transceivers (~$2 each) but well worth the robustness.

---

## 5. Power Source Selection

**Baseline (S):** Concept A — 3S LiPo 11.1V 5000mAh

| Criteria | Weight | Concept A: 3S 5000mAh (Baseline) | Concept B: 4S 3000mAh | Concept C: NiMH 12V |
|----------|--------|----------------------------------|----------------------|---------------------|
| Voltage compatibility (12V motors) | 0.25 | S | - | S |
| Runtime | 0.20 | S | - | - |
| Weight | 0.15 | S | + | - |
| Energy density | 0.15 | S | S | - |
| Safety (LiPo vs NiMH) | 0.10 | S | S | + |
| Availability/cost | 0.10 | S | S | - |
| Charging simplicity | 0.05 | S | S | S |
| **Weighted Score** | | **0 (baseline)** | **-0.15** | **-0.30** |

**Selected: Concept A — 3S LiPo 11.1V 5000mAh**

Rationale: 11.1V nominal is close enough to 12V motor rating. 5000mAh provides ~1.4 hours runtime. Good balance of weight, capacity, and cost. Widely available from RC hobby suppliers.

---

## 6. Summary of Selected Concepts

| Function | Selected Concept | Key Advantage |
|----------|-----------------|---------------|
| Locomotion | 4x Mecanum rectangular | Stability + direct Pure X/Y/Rotate mapping |
| Arm | 4-DOF serial articulated | Simple IK + sufficient reach + low cost |
| Gripper | Servo 2-finger parallel | Simple, reliable for 5 cm cubes |
| Computing | ESP32-S3 x2 + RPi 4B | Real-time control + reliable vision |
| Vision | RPi camera + pyzbar + HSV | High QR accuracy + color backup |
| Communication | CAN bus (TWAI) + UART | Robust modular link + simple RPi interface |
| Power | 3S LiPo 11.1V 5000mAh | Voltage match + ample capacity |
| Frame | Aluminum extrusion + 3D print | Stiff base + custom arm parts |
| Odometry | Complementary filter (encoder + IMU) | Simple + sufficient accuracy |
