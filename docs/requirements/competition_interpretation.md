# Competition Interpretation — Spring 2026 Mechatronics Omni-Challenge

**Purpose:** Detailed breakdown of competition rules, scoring, arena layout, and strategic implications for robot design.

---

## 1. Arena and Track Layout

### Dimensions
- **Total track length:** ~10 m defined path
- **Manual Control Zone:** 10 m x 5 m (start area)
- **Track lane width:** 0.8 m (two parallel black boundary lines)
- **Floor surface:** Concrete with vinyl tape markings

### Key Zones
1. **Start Box** — Robot begins here, arm folded within dimension limits
2. **Manual Control Zone** — Gamepad-operated driving area
3. **Transition Line** — Handover point from manual to autonomous
4. **Ball Pit Area / Pickup Area** — Where cubes are located on pedestals (~40 cm height)
5. **Drop-off Stations** — Three stations (Red, Blue, Green) with constrained access

### Pickup Stations (Pedestals)
- **Station A (RED):** Requires Pure Y movement (forward/backward only)
- **Station B (BLUE):** Requires Pure X movement (lateral strafe only)
- **Station C (GREEN):** Requires Rotate-in-place movement

### Obstacles
- May be placed near the track lane (details TBD by instructors)
- Robot should have basic obstacle awareness

---

## 2. Competition Procedure

### Pre-Competition: Safety Gate (Pass/Fail)
Must pass ALL of these to compete:
1. No hanging/exposed wires (proper harnessing, strain relief)
2. No sharp edges or unsafe protrusions (deburr and cover)
3. Weight within 10 kg limit
4. Dimensions within 500x500x700 mm (arm folded)
5. Main fuse installed + emergency stop / accessible power cutoff
6. Electronics professionally mounted (PCB in enclosure, no loose boards)

**Strategy:** Prepare a safety checklist and do internal inspections from Week 7 onward.

### Phase 1: Manual Teleoperation
- Robot starts fully inside start box within folded-dimension limits
- Operator drives using wireless controller through start area
- Position robot near the ball pit / pickup area
- **Handover:** Operator places controller down, robot switches to autonomous mode
- **Scoring:** 10 points for reaching and stopping at transition line

### Phase 2: Autonomous Handling
- **No human intervention allowed** for picking/dropping boxes
- Robot reads QR code → identifies color → prints color on laptop screen
- Picks up cube autonomously (5 points per box)
- After picking, manually navigate to correct station (Red/Blue/Green)
- Execute required maneuver at station:
  - Red → Pure Y (forward/backward)
  - Blue → Pure X (lateral strafe)
  - Green → Rotate in place
- Encoder + IMU readings displayed on laptop during maneuvers
- **Scoring:** 10 points per correct maneuver + 10 points per correct delivery

---

## 3. Scoring Breakdown

| Event | Points | Notes |
|-------|--------|-------|
| Safety inspection passed | Mandatory gate | No points, but required to compete |
| Manual phase: reach transition line | 10 | Stop cleanly at the line |
| Autonomous pick per box | 5 | Robot picks cube without human help |
| Correct on-board color sorting | +2 bonus per box | Only if placed in correct bin |
| Correct maneuvering to drop-off zone | 10 per box | Execute Pure Y/X/Rotate correctly |
| Correct delivery to matching zone | 10 per box | Cube fully inside target square |

### Maximum Score (3 cubes)
| Component | Points |
|-----------|--------|
| Manual phase | 10 |
| Autonomous pick (3 cubes) | 15 |
| Color sorting bonus (3 cubes) | 6 |
| Correct maneuvering (3 zones) | 30 |
| Correct delivery (3 cubes) | 30 |
| **Total Maximum** | **91** |

### Penalties
| Penalty | Points |
|---------|--------|
| Lane boundary crossing | -3 each |
| Collision with obstacle | -5 each |
| Manual intervention after transition | -10 (repeat may invalidate run) |

### Tie-Breaker (in order)
1. Highest score
2. Fastest valid time
3. Lightest robot

---

## 4. Strategic Design Implications

### Priority 1: Reliability over Speed
- The scoring heavily rewards **completing tasks correctly** rather than completing them fast
- A reliable 2-cube run scores more than a fast but failed 3-cube attempt
- Focus on repeatability: target ≥80% success rate before optimizing speed

### Priority 2: Pure X/Y/Rotate Precision
- 30 points come from correct maneuvering — this is the largest single category
- Mecanum wheel kinematic accuracy is critical
- IMU heading stabilization prevents drift during strafing
- Encoder-based distance control ensures accurate travel

### Priority 3: QR Detection Reliability
- 15 points (pick) + 6 bonus (sort) depend on QR reading
- Test under varied lighting conditions
- Implement backup color detection if QR fails

### Priority 4: Lightweight Design
- Robot weight is a tie-breaker criterion
- Current estimate ~4 kg is well under 10 kg, giving advantage in tie situations
- Don't add unnecessary weight — every gram counts in tie-breaks

### Priority 5: Manual Phase Speed
- Only 10 points for reaching transition line
- Don't over-optimize teleop — basic reliable driving is sufficient
- Practice operator driving a few times

---

## 5. Time Budget (5-minute run)

| Phase | Estimated Time | Notes |
|-------|---------------|-------|
| Manual drive to transition | ~30-60 s | Simple path, operator controlled |
| Autonomous QR read + pick (per cube) | ~15-20 s | Detection + approach + grasp |
| Manual drive to station | ~20-30 s | Per station |
| Constrained maneuver + delivery | ~15-20 s | Per station |
| **Total for 3 cubes** | **~3-4 min** | Leaves ~1 min buffer |

This time budget is achievable but tight. Focus on reliable single-cube execution first, then extend to multiple cubes.

---

## 6. Critical Display Requirements

The competition explicitly requires certain data displayed on a laptop screen:
1. **Detected box color** — printed after QR code reading (during autonomous pick phase)
2. **Encoder readings** — displayed during constrained maneuvers at stations
3. **IMU readings** — displayed during constrained maneuvers at stations

**Implementation:** Python GUI on RPi (or connected laptop) showing real-time telemetry. Can be a simple terminal printout or a basic dashboard.
