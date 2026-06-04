![Canal Gate Automation System](https://img.shields.io/badge/Canal%20Gate%20Automation%20System-Silicon%20to%20Cloud-8f7fe0?style=for-the-badge&labelColor=171129)

> **Dual-gate canal automation system — ESP8266 reads water levels via HC-SR04 ultrasonic sensors and drives custom 3D-printed motorized gates through L298N H-bridges. Full Blynk IoT remote control with automatic and manual modes.**

---

![ESP8266](https://img.shields.io/badge/ESP8266-NodeMCU-8f7fe0?style=flat-square&logo=espressif&logoColor=white&labelColor=171129)
![C++](https://img.shields.io/badge/C++-Firmware-8f7fe0?style=flat-square&logo=cplusplus&logoColor=white&labelColor=171129)
![Blynk](https://img.shields.io/badge/Blynk-IoT%20Cloud-8f7fe0?style=flat-square&logo=blynk&logoColor=white&labelColor=171129)
![HC-SR04](https://img.shields.io/badge/HC--SR04-Ultrasonic%20Sensor-8f7fe0?style=flat-square&labelColor=171129)
![L298N](https://img.shields.io/badge/L298N-H--Bridge%20Driver-8f7fe0?style=flat-square&labelColor=171129)
![3D Printing](https://img.shields.io/badge/3D%20Printing-Custom%20Gates-8f7fe0?style=flat-square&logo=printables&logoColor=white&labelColor=171129)
![Arduino IDE](https://img.shields.io/badge/Arduino%20IDE-Firmware%20Dev-8f7fe0?style=flat-square&logo=arduino&logoColor=white&labelColor=171129)

---

## Executive Summary

Manual irrigation gate control wastes water and labor at scale. This system automates it end-to-end — two HC-SR04 sensors continuously measure canal water levels, an ESP8266 runs threshold logic to decide gate position, and custom 3D-printed motorized gates respond in real time, all visible and controllable from a Blynk mobile dashboard anywhere in the world.

---

## System Architecture

<img src="assets/how_it_works.png" width="100%" alt="System Architecture — Silicon to Cloud"/>

---

## The Problem This Solves

Traditional irrigation canal gates are operated manually — a person physically opens or closes a gate based on visual inspection. This approach fails at scale: it requires constant human presence, reacts slowly to water level changes, and has no remote visibility or override capability. This system replaces the human loop entirely with sensor-driven automation and cloud-connected control, while keeping a manual override available for edge cases.

| Approach | Response Time | Remote Control | Automation |
|---|---|---|---|
| Manual gate operation | Minutes to hours | ✗ | ✗ |
| This system | < 3 seconds | ✓ Blynk IoT | ✓ Threshold logic |

---

## Technical Stack

### Hardware — Silicon Layer

| Component | Spec | Role |
|---|---|---|
| ESP8266 NodeMCU | 80MHz, 802.11 b/g/n WiFi | Main MCU — firmware execution, WiFi, Blynk client |
| HC-SR04 × 2 | 2cm–400cm range, ±3mm accuracy | Water level sensing — downward-facing above canal |
| DC Gear Motor × 2 | 5V–12V DC | Gate actuation via lead-screw mechanism |
| L298N H-Bridge × 2 | 2A per channel, dual channel | Motor direction and speed control |
| Custom 3D-Printed Gates | PLA — lead screw + linear rails | Physical gate body, frame, and travel mechanism |
| Breadboard + wiring | — | Prototyping and circuit integration |

### Firmware

![C++](https://img.shields.io/badge/C++-Arduino%20Framework-8f7fe0?style=flat-square&logo=cplusplus&logoColor=white&labelColor=171129)
![Blynk Library](https://img.shields.io/badge/BlynkSimpleEsp8266-v1.x-8f7fe0?style=flat-square&labelColor=171129)
![ESP8266WiFi](https://img.shields.io/badge/ESP8266WiFi-Core%20Library-8f7fe0?style=flat-square&logo=espressif&logoColor=white&labelColor=171129)

Custom firmware written in C++ on the Arduino framework. Handles dual sensor polling at 10 reads/sec, percentage mapping with inverted distance logic, hysteresis-band threshold engine, motor timing guards, and full bidirectional Blynk sync.

### Cloud Backend

![Blynk IoT](https://img.shields.io/badge/Blynk-IoT%20Platform-8f7fe0?style=flat-square&logo=blynk&logoColor=white&labelColor=171129)

Blynk IoT handles the cloud bridge — the ESP8266 pushes live water level percentages to virtual pin gauges and receives mode toggle and gate override commands from the mobile app in real time.

### Application Layer

![Blynk Mobile](https://img.shields.io/badge/Blynk%20App-Android%20%2F%20iOS-8f7fe0?style=flat-square&logo=blynk&logoColor=white&labelColor=171129)

Blynk mobile dashboard — live water level gauges for both gates, automation ON/OFF toggle, and individual gate open/close override buttons. Zero custom app code required; all logic lives in the firmware.

---

## Engineering Deep Dive

### The Inverted Sensor Problem

The HC-SR04 sensors mount above the water surface pointing downward. This means the raw distance reading is inversely proportional to water level — a short distance means high water, a long distance means low water. The firmware corrects this with a single map() inversion:

```cpp
// Raw distance → water level percentage (inverted)
// MIN_DISTANCE (24mm) = sensor closest = 100% full
// MAX_DISTANCE (92mm) = sensor farthest = 0% empty
int sensorPercentage = map(distance_mm, MIN_DISTANCE, MAX_DISTANCE, 100, 0);
```

Without this correction, every gate decision would be reversed.

### Hysteresis Band — Preventing Motor Chatter

A naive threshold implementation (open if < 60%, close if > 60%) causes continuous motor oscillation when water level hovers near the trigger point. The solution is a hysteresis band with two separate thresholds:

```cpp
#define OPEN_GATE_THRESHOLD   70   // Close gate above this level
#define CLOSE_GATE_THRESHOLD  50   // Open gate below this level
// Band between 50–70%: no action — gate holds last position
```

This creates a 20% dead band. Once the gate opens (level < 50%), it won't close again until the level rises above 70%. This eliminates rapid toggling and reduces mechanical wear.

### Motor Rest Guard — Mechanical Debounce

Even with hysteresis, rapid sequential commands could damage the lead-screw mechanism. A timing guard enforces a minimum rest period between activations, and direction memory prevents re-issuing the same command:

```cpp
if (openGateCondition &&
    (currentTime - lastMoveTime) > MOTOR_REST_DURATION &&  // 2s minimum rest
    lastDir != BACKWARD) {                                   // Not already open
    moveMotorBackward(motorIn1, motorIn2);
    delay(MOTOR_MOVE_DURATION);   // 1s actuation window
    stopMotor(motorIn1, motorIn2);
    lastMoveTime = millis();
    lastDir = BACKWARD;
}
```

### Auto / Manual Mode Switching

The Blynk V2 toggle switches between modes cleanly. In manual mode, BLYNK_WRITE handlers accept gate commands directly. In auto mode, those same handlers ignore incoming commands — the sensor logic overwrites the button state instead:

```cpp
BLYNK_WRITE(VIRTUAL_PIN_GATE1) {
  if (!automationEnabled) {          // Only act in manual mode
    manualGate1State = param.asInt();
    controlGate(manualGate1State, MOTOR1_IN1, MOTOR1_IN2);
  }
  // In auto mode: silently ignored. Sensor logic controls the button.
}
```

---

## Features

- **Dual Independent Gate Control** — each gate has its own sensor, motor, and logic pipeline
- **Real-Time Blynk IoT Dashboard** — live water level percentages, gate status, mode toggle
- **Automatic Mode** — threshold + hysteresis engine drives gates without human input
- **Manual Override** — full gate control from the Blynk app when automation is off
- **Motor Protection** — 2s rest guard and direction memory prevent mechanical damage
- **Custom 3D-Printed Gates** — lead-screw linear actuator mechanism, fully fabricated
- **Calibration-Ready** — all thresholds, timing constants, and pin assignments in one config block
- **Serial Debug Output** — labeled boot, sensor, motor, and mode events for on-bench diagnostics

---

## UI / Demo

### Main System — Final Diorama Build

<img src="assets/main.png" width="100%" alt="Canal Gate Automation System — Final Build"/>

### 3D-Printed Gate Mechanisms

<img src="assets/3d_printed_gates+manual_gate.png" width="100%" alt="3D Printed Gates — Manual frame (left), motorized lead-screw gates (center and right)"/>

*Left: manual gate frame. Centre: motorized gate with green 3D-printed lead-screw carriage. Right: second motorized gate in black/white.*

### Technical Stack

<img src="assets/technical_data.png" width="100%" alt="Technical Details — ESP8266, HC-SR04, L298N, Blynk IoT"/>

### Firmware — Arduino IDE

<img src="assets/code.png" width="100%" alt="Firmware running on NodeMCU 1.0 ESP-12E in Arduino IDE"/>

### Prototype Build Stages

<table>
  <tr>
    <td width="50%"><img src="assets/model_raw_1.jpg" width="100%" alt="Prototype Stage 1 — water container test rig"/></td>
    <td width="50%"><img src="assets/model_raw_2.jpg" width="100%" alt="Prototype Stage 2 — wiring and breadboard"/></td>
  </tr>
  <tr>
    <td align="center"><em>Prototype stage — Gate 1 side, transparent container water test</em></td>
    <td align="center"><em>Prototype stage — Gate 2 side, full breadboard wiring visible</em></td>
  </tr>
</table>

---

## Project Context

Built in 2026 as part of my engineering portfolio. Built to demonstrate a complete hardware-to-cloud automation pipeline — sensor reading, firmware decision logic, physical actuation, and remote IoT control — using off-the-shelf components and custom fabricated mechanical parts.

---

## Contact

[![Email](https://img.shields.io/badge/Email-roshaanahsan.pro%40gmail.com-8f7fe0?style=flat-square&logo=gmail&logoColor=white&labelColor=171129)](mailto:roshaanahsan.pro@gmail.com)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-roshaanahsan-8f7fe0?style=flat-square&logo=linkedin&logoColor=white&labelColor=171129)](https://linkedin.com/in/roshaanahsan)
[![GitHub](https://img.shields.io/badge/GitHub-roshaanahsan-8f7fe0?style=flat-square&logo=github&logoColor=white&labelColor=171129)](https://github.com/roshaanahsan)
[![X](https://img.shields.io/badge/X-roshaanahsan-8f7fe0?style=flat-square&logo=x&logoColor=white&labelColor=171129)](https://x.com/roshaanahsan)
