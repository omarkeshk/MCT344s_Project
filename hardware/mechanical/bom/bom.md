# Bill of Materials — Modular Omni-Wheel Mobile Manipulator

**Version:** 1.0 | **Updated:** Week 0

---

## 1. Drive System

| # | Component | Specification | Qty | Unit Price (est.) | Total | Supplier | Notes |
|---|-----------|--------------|-----|-------------------|-------|----------|-------|
| 1 | DC Gear Motor with Encoder | JGA25-371, 12V, 200RPM, hall-effect quadrature encoder | 4 | $10 | $40 | AliExpress / Amazon | Built-in 11 PPR encoder, 25mm gearbox |
| 2 | Mecanum Wheel Set | 60mm diameter, 2L + 2R set, 4mm/6mm shaft bore | 1 set (4 pcs) | $35 | $35 | Amazon / RobotShop | Ensure L/R pair match motor positions |
| 3 | Motor-to-Wheel Coupler | 4mm motor shaft to 4mm wheel bore, rigid or set-screw | 4 | $2 | $8 | Amazon / AliExpress | Match shaft diameters |
| 4 | Dual H-Bridge Motor Driver | TB6612FNG breakout, 1.2A/ch continuous, 3.2A peak | 2 | $4 | $8 | Amazon / Adafruit | Each drives 2 motors |

**Subtotal: ~$91**

---

## 2. Manipulator (Arm + Gripper)

| # | Component | Specification | Qty | Unit Price (est.) | Total | Supplier | Notes |
|---|-----------|--------------|-----|-------------------|-------|----------|-------|
| 5 | Servo — Base Yaw (J1) | MG996R, 10 kg-cm, metal gear, standard size | 1 | $5 | $5 | Amazon / AliExpress | |
| 6 | Servo — Shoulder (J2) | DS3218, 20 kg-cm, metal gear, digital | 1 | $10 | $10 | Amazon / AliExpress | Highest torque joint |
| 7 | Servo — Elbow (J3) | MG996R, 10 kg-cm, metal gear | 1 | $5 | $5 | Amazon / AliExpress | |
| 8 | Servo — Wrist (J4) | MG90S, 2.2 kg-cm, metal gear, micro | 1 | $3 | $3 | Amazon / AliExpress | |
| 9 | Servo — Gripper (J5) | SG90, 1.8 kg-cm, plastic gear, micro | 1 | $2 | $2 | Amazon / AliExpress | |
| 10 | Servo Horn / Bracket Kit | Aluminum U-bracket + servo horns for arm assembly | 1 set | $15 | $15 | Amazon / AliExpress | Or 3D print custom brackets |
| 11 | Gripper Mechanism | 2-finger parallel gripper kit (compatible with SG90/MG90S) | 1 | $8 | $8 | Amazon / AliExpress | Or 3D print custom design |

**Subtotal: ~$48**

---

## 3. Controllers and Computing

| # | Component | Specification | Qty | Unit Price (est.) | Total | Supplier | Notes |
|---|-----------|--------------|-----|-------------------|-------|----------|-------|
| 12 | ESP32-S3-DevKitC-1 | N16R8 (16MB Flash, 8MB PSRAM) | 1 | $12 | $12 | Amazon / Mouser / AliExpress | Base controller |
| 13 | ESP32-S3-DevKitC-1 | N8R2 (8MB Flash, 2MB PSRAM) | 1 | $10 | $10 | Amazon / Mouser / AliExpress | Arm controller |
| 14 | Raspberry Pi 4B | 4GB RAM | 1 | $60 | $60 | Amazon / RPi reseller | Supervisory controller |
| 15 | Raspberry Pi Power Supply | USB-C 5V/3A (for bench testing only) | 1 | $10 | $10 | Amazon | Not used on robot (buck converter instead) |
| 16 | MicroSD Card | 32GB Class 10 for RPi OS | 1 | $8 | $8 | Amazon | |

**Subtotal: ~$100**

---

## 4. Sensors

| # | Component | Specification | Qty | Unit Price (est.) | Total | Supplier | Notes |
|---|-----------|--------------|-----|-------------------|-------|----------|-------|
| 17 | IMU Module | MPU6050 (or ICM-20948 for better accuracy), I2C breakout | 1 | $5 | $5 | Amazon / AliExpress | Heading stabilization |
| 18 | Camera | RPi Camera Module v3 (CSI) or Logitech C270 (USB) | 1 | $30 | $30 | Amazon / RPi reseller | QR detection + color classification |
| 19 | Proximity Sensor (optional) | VL53L0X Time-of-Flight, I2C, 2m range | 2 | $5 | $10 | Amazon / AliExpress | Front obstacle detection |

**Subtotal: ~$45**

---

## 5. Power System

| # | Component | Specification | Qty | Unit Price (est.) | Total | Supplier | Notes |
|---|-----------|--------------|-----|-------------------|-------|----------|-------|
| 20 | LiPo Battery | 3S 11.1V 5000mAh 50C, XT60 connector | 1 | $35 | $35 | Amazon / HobbyKing | Main power source |
| 21 | LiPo Battery Charger | 3S balance charger (e.g., IMAX B6) | 1 | $25 | $25 | Amazon | If team doesn't have one already |
| 22 | Buck Converter (RPi) | MP1584 or LM2596, 12V→5V, 3A output | 1 | $3 | $3 | Amazon / AliExpress | Dedicated RPi power |
| 23 | Buck Converter (Servos) | LM2596, 12V→5V, 3A output | 1 | $3 | $3 | Amazon / AliExpress | Separate rail to avoid noise |
| 24 | Main Power Switch | Toggle switch, 10A rated | 1 | $3 | $3 | Amazon | Accessible on robot exterior |
| 25 | Main Fuse + Holder | Blade fuse 15A + inline holder | 1 | $3 | $3 | Amazon / auto parts | Safety requirement |
| 26 | XT60 Connector Pair | Male/female for battery connection | 2 | $2 | $4 | Amazon / AliExpress | |
| 27 | LiPo Voltage Alarm | 1S-3S low voltage buzzer | 1 | $2 | $2 | Amazon / AliExpress | Audible low-battery warning |

**Subtotal: ~$78**

---

## 6. Communication and Wiring

| # | Component | Specification | Qty | Unit Price (est.) | Total | Supplier | Notes |
|---|-----------|--------------|-----|-------------------|-------|----------|-------|
| 28 | CAN Transceiver Module | SN65HVD230 breakout board, 3.3V | 2 | $3 | $6 | Amazon / AliExpress | One per ESP32 |
| 29 | USB-A to Micro-USB Cable | 30cm, for RPi to ESP32 connection | 2 | $3 | $6 | Amazon | Short cables for on-robot use |
| 30 | GX16-6 Aviation Connector | 6-pin male+female panel mount pair | 1 pair | $5 | $5 | Amazon / AliExpress | Modular interface connector |
| 31 | Wire Kit | 18-22 AWG stranded, assorted colors, silicone | 1 set | $12 | $12 | Amazon | For power and signal wiring |
| 32 | Dupont Jumper Wires | Male-Male, Male-Female, Female-Female sets | 1 set | $5 | $5 | Amazon | For prototyping |
| 33 | Heat Shrink Tubing | Assorted sizes kit | 1 set | $5 | $5 | Amazon | Wire insulation |
| 34 | Cable Ties + Mounts | Zip ties (assorted) + adhesive mounts | 1 set | $5 | $5 | Amazon | Cable management (safety req) |
| 35 | Breadboard (prototyping) | Half-size solderless breadboard | 2 | $3 | $6 | Amazon | Initial prototyping |

**Subtotal: ~$50**

---

## 7. Operator Interface

| # | Component | Specification | Qty | Unit Price (est.) | Total | Supplier | Notes |
|---|-----------|--------------|-----|-------------------|-------|----------|-------|
| 36 | Wireless Gamepad | PS4 DualShock (Bluetooth) or Logitech F710 (USB dongle) | 1 | $25 | $25 | Amazon | Bluetooth pairs with RPi |

**Subtotal: ~$25**

---

## 8. Frame and Structure

| # | Component | Specification | Qty | Unit Price (est.) | Total | Supplier | Notes |
|---|-----------|--------------|-----|-------------------|-------|----------|-------|
| 37 | 3D Printing Filament | PETG, 1kg spool (for arm links, brackets, enclosures) | 1 | $20 | $20 | Amazon | PETG preferred over PLA for strength |
| 38 | Aluminum Extrusion | 2020 V-slot, 300mm lengths | 4 | $4 | $16 | Amazon / AliExpress | Base frame structure |
| 39 | Corner Brackets | 2020 L-brackets, zinc alloy | 8 | $1 | $8 | Amazon / AliExpress | Frame assembly |
| 40 | T-Nuts + Bolts for 2020 | M4/M5 T-slot nuts + bolts, assorted kit | 1 set | $8 | $8 | Amazon / AliExpress | |
| 41 | Aluminum Plate | 3mm thick, ~200x200mm for top plate / arm mount | 1 | $8 | $8 | Amazon / local | Arm mounting plate |
| 42 | M3/M4 Fastener Kit | Bolts, nuts, washers, standoffs assorted | 1 set | $10 | $10 | Amazon | General assembly |
| 43 | Rubber Pads / Bumpers | Self-adhesive, for base underside | 1 pack | $3 | $3 | Amazon | Vibration dampening |

**Subtotal: ~$73**

---

## Summary

| Category | Subtotal |
|----------|----------|
| Drive System | $91 |
| Manipulator | $48 |
| Controllers & Computing | $100 |
| Sensors | $45 |
| Power System | $78 |
| Communication & Wiring | $50 |
| Operator Interface | $25 |
| Frame & Structure | $73 |
| **Grand Total** | **~$510** |

---

## Priority Ordering (Order by Week 2)

**Critical path items (order immediately):**
1. JGA25-371 motors (#1) — 1-2 week shipping from AliExpress
2. Mecanum wheel set (#2) — often ships with motors
3. ESP32-S3 boards (#12, #13) — development can't start without these
4. Raspberry Pi 4B (#14) — perception development depends on this
5. LiPo battery (#20) + charger (#21) — needed for any powered testing
6. Servos (#5-9) — arm testing depends on these

**Can order later (Week 4-5):**
- CAN transceivers, connectors, wiring supplies
- Camera (can prototype with laptop webcam initially)
- Aluminum extrusion and structural materials
- 3D printing filament (if university lab provides)

---

## Notes

- Prices are estimates and may vary by supplier and region
- Consider buying 1-2 spare ESP32 boards and spare motors
- LiPo batteries require safe handling — store in fireproof bag, never over-discharge
- Check if university lab can provide 3D printing, laser cutting, or aluminum stock
