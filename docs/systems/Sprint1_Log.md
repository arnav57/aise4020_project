# System Architecture + Functions (Sprint 1)

---

## 1) System Goal

Develop an autonomous vehicle (PiCar-X–style platform) that can:

- Follow lane markings using a hybrid sensing approach (camera + grayscale)
- Detect and obey posted speed limits (real numeric values)
- Stop safely for obstacles
- Obey one infrastructure stop rule (traffic light (easier) OR stop sign)
- Scale to more advanced behaviors (e.g., lane switching) without redesign

Sprint 1 defines the architecture, function set, and integration contracts.
Implementation begins in Sprint 2.

---

## 2) Platform Assumptions (Hardware-Realistic)

Target platform characteristics (approximate):
- Raspberry Pi 4/5 (NOT INCLUDED)
- Camera (5MP class)
- Ultrasonic distance sensor
- Grayscale line sensors
- 2 motors (rear drive)
- 1 steering servo (front steering)
- Motor driver / Robot HAT

Key implication:
Control outputs are **motor_speed + steering_angle**, not left/right PWM steering.

---

# 3) Locked MVP Functions

These functions MUST work before any expansion.

---

## F1 — Lane Keeping (Hybrid: Camera + Grayscale)

The vehicle maintains lane center using a hybrid sensing strategy:
- Camera provides lane geometry and robustness when line sensors struggle - Backup
- Grayscale sensors provide stable line-following when vision confidence drops - MAIN

Primary Sensors:
- Camera + Grayscale (hybrid)

Perception Outputs (lane):
- Distance from centre

Fusion Rule (Sprint 1 definition; implemented later):
- If grayscale confidence is high → use gray lane_offset
- Else → fall back to cam lane_offset
- If both confidence low → safe stop / slow crawl (project-defined)

Planning Output:
- steering_command (continuous)

Control Output:
- steering_angle (servo target)

Success Criteria:
- Vehicle stays in-lane for repeated short runs on a controlled track.

---

## F2 — Obstacle Safety Stop (Safety Layer)

The vehicle stops when an obstacle is within a defined threshold distance.

Primary Sensor:
- Ultrasonic

Perception Output:
- obstacle_distance_cm (float or None)

Planning Behavior:
- If obstacle_distance_cm < threshold → STOP_OBSTACLE state (highest priority after emergency)

Control Output:
- motor_speed = 0 (brake override)

Success Criteria:
- Reliable stop response regardless of lane tracking mode.

---

## F3 — Speed Limit Detection and Compliance (Real Numeric)

The vehicle detects numeric speed limit signs (e.g., 10, 20, 30 km/h) and adjusts its target speed.

Parts Necessary:
- Wheels w/ encoders

Primary Sensor:
- Camera

Perception Output:
- speed_limit_kmh (int or None)
- speed_limit_confidence (0–1)

Planning Behavior:
- Maintain an active speed limit value
- Convert speed_limit_kmh → target_speed (scaled motor_speed or target m/s)
- Enforce: commanded speed ≤ active speed limit


Success Criteria:
- Vehicle visibly reduces speed when a lower limit is detected.
- Vehicle does not command higher speed than the active speed limit.

---

## F4 — Stop Stop Behavior (or traffic light)

Option A:
- Traffic light stop/go

Option B:
- Stop sign behavior

Primary Sensor:
- Camera

Perception Output:
- traffic_state ("RED/GREEN/UNKNOWN")
OR
- stop_sign_state ("STOP/NONE/UNKNOWN")

Success Criteria:
- Vehicle stops correctly at the controlled infrastructure element and resumes appropriately.

---

# 4) Future Feature

---

## F5 — Lane Switching (Stretch)

Lane switching is not required for MVP but supported by the architecture.

Requires:
- adjacent lane detection (camera-based, wide-angled camera) OR track markers
- safety check before switching (no obstacle)
- controlled transition + re-acquisition of lane center

Planning additions:
- states: LANE_CHANGE_LEFT, LANE_CHANGE_RIGHT
- command input: lane_change_cmd ("NONE/LEFT/RIGHT")

Not attempted unless MVP functions are stable.

---

# 5) High-Level Architecture (Layered)

Sensing → Perception → Planning (FSM) → Control → Actuation

---

## Sensing Layer
- Camera frames
- Ultrasonic readings
- Grayscale sensor readings

---

## Perception Layer
Produces structured outputs:
- lane_offset_cam, lane_confidence_cam
- lane_offset_gray, lane_confidence_gray
- obstacle_distance_cm
- speed_limit_kmh, speed_limit_confidence
- traffic_state OR stop_sign_state
- timestamp

---

## Planning Layer (Finite State Machine)
Responsible for:
- selecting driving mode based on priorities
- selecting lane source (camera vs grayscale) based on confidence
- applying speed limit constraints

Priority Order (Highest → Lowest):
1. EMERGENCY_STOP
2. STOP_OBSTACLE
3. STOP_TRAFFIC / STOP_SIGN
4. SPEED_LIMIT_ENFORCEMENT
5. LANE_FOLLOW (hybrid lane tracking)
6. LANE_CHANGE (stretch)

Planning Outputs:
- state
- target_speed (scaled / normalized)
- steering_command (-1..+1 or angle target)
- timestamp

Notes:
- STOP states override speed commands entirely (target_speed = 0).
- Speed limit modifies target_speed while moving, but does not override stop states.

---

## Control Layer (Platform-Specific)
Converts:
- target_speed → motor_speed command (0–1)
- steering_command → steering_angle (servo)

Control Outputs:
- motor_speed (0–1)
- steering_angle (normalized or degrees)
- brake (bool)
- timestamp

---

## Actuation Layer
- Motor driver / Robot HAT
- Rear motors
- Steering servo
- Optional indicators (LED/Buzzer)

---

# 6) Feature-to-Sensor Mapping (Primary + Fallback)

| Function | Primary Sensor(s) | Fallback |
|---------|--------------------|----------|
| Lane Keeping | Camera + Grayscale | Use other source based on confidence |
| Obstacle Stop | Ultrasonic | None (failsafe stop) |
| Speed Limit Detection | Camera | Simplified marker if needed |
| Traffic/Stop Rule | Camera | Marker-based fallback |
| Lane Switching | Camera + obstacle check | Disabled if unstable |

---

# 7) Required Integration Contracts (Module Outputs)

Perception must output:
- lane_offset_cam (float)
- lane_confidence_cam (0–1)
- lane_offset_gray (float)
- lane_confidence_gray (0–1)
- obstacle_distance_cm (float or None)
- speed_limit_kmh (int or None)
- speed_limit_confidence (0–1)
- traffic_state OR stop_sign_state
- timestamp

Planning must output:
- state
- target_speed (0–1) OR target_speed_mps (float)
- steering_command (normalized -1..+1) OR steering_angle_target
- timestamp

Control must output:
- motor_speed (0–1)
- steering_angle (normalized or degrees)
- brake (bool)
- timestamp

---

# 8) Sprint 1 Completion Criteria (Ready for Sprint 2)

Sprint 1 is complete when:
- MVP functions (F1–F4) are defined and locked
- Hybrid lane tracking strategy is defined (confidence-based switching)
- Planning priorities are finalized
- Required perception/planning/control outputs are specified
- Repository structure supports modular implementation