/**
 * ============================================================
 *  CANAL GATE AUTOMATION SYSTEM
 *  Automatic Water Gate Controller
 * ============================================================
 *
 *  Author  : Roshaan Ahsan
 *  GitHub  : github.com/roshaanahsan
 *  Contact : roshaanahsan.pro@gmail.com
 *
 * ============================================================
 *  PROJECT OVERVIEW
 * ============================================================
 *
 *  This firmware runs on an ESP8266 (NodeMCU) to automate the
 *  control of two motorized water gates in an irrigation canal
 *  system. Two HC-SR04 ultrasonic sensors continuously measure
 *  the water level in their respective sections. Based on those
 *  readings, the firmware decides whether to raise or lower each
 *  gate using a DC motor + H-bridge driver (L298N).
 *
 *  The entire system is monitored and controlled in real time
 *  through the Blynk IoT mobile app — operators can switch
 *  between automatic and manual modes, and override individual
 *  gates from anywhere with an internet connection.
 *
 * ============================================================
 *  HARDWARE SUMMARY
 * ============================================================
 *
 *  MCU          : ESP8266 (NodeMCU v3 or equivalent)
 *  Sensors      : 2x HC-SR04 Ultrasonic Distance Sensors
 *  Actuators    : 2x DC Gear Motors driving lead-screw gate
 *                 mechanisms (custom 3D-printed hardware)
 *  Motor Driver : 2x L298N H-Bridge modules (one per gate)
 *  Cloud        : Blynk IoT Platform (Legacy or New)
 *  Power        : 5V USB for ESP8266; external 12V for motors
 *
 * ============================================================
 *  SYSTEM ARCHITECTURE (Silicon → Cloud)
 * ============================================================
 *
 *  [Water Level]
 *      │
 *      ▼
 *  [HC-SR04 Ultrasonic Sensor]  ←── measures distance to
 *      │                              water surface in mm
 *      ▼
 *  [ESP8266 Firmware]           ←── converts distance to %,
 *      │                              applies threshold logic
 *      ├──► [L298N Motor Driver] ──► [DC Motor] ──► [Gate]
 *      │
 *      └──► [Blynk Cloud] ──► [Mobile App]
 *                │                 │
 *                │                 ├── Live water level gauges
 *                │                 ├── Auto / Manual toggle
 *                └─────────────────└── Manual gate override
 *
 * ============================================================
 *  GATE LOGIC (INVERTED MOUNTING)
 * ============================================================
 *
 *  The ultrasonic sensors are mounted ABOVE the water, pointing
 *  DOWN. This means a SMALL distance reading = HIGH water level,
 *  and a LARGE distance reading = LOW water level.
 *
 *  The firmware compensates for this with inverted logic:
 *
 *    Water level %  =  map(distance, MIN_MM, MAX_MM, 100, 0)
 *                                                    ↑    ↑
 *                                              close  far
 *
 *  Gate behaviour:
 *    • Level < CLOSE_GATE_THRESHOLD (50%) → water is LOW  → OPEN gate
 *    • Level > OPEN_GATE_THRESHOLD  (70%) → water is HIGH → CLOSE gate
 *
 *  A hysteresis band between 50% and 70% prevents rapid
 *  motor oscillation when the water level hovers near a threshold.
 *
 * ============================================================
 *  BLYNK VIRTUAL PIN MAP
 * ============================================================
 *
 *  V0  →  Sensor 1 water level (0–100 %)   [display widget]
 *  V1  →  Sensor 2 water level (0–100 %)   [display widget]
 *  V2  →  Automation ON / OFF toggle        [button widget]
 *  V3  →  Gate 1 state / manual toggle      [button widget]
 *  V4  →  Gate 2 state / manual toggle      [button widget]
 *
 * ============================================================
 *  HOW TO CONFIGURE
 * ============================================================
 *
 *  1. Create a Blynk template and copy its credentials below.
 *  2. Fill in your WiFi SSID and password.
 *  3. Adjust MIN_DISTANCE / MAX_DISTANCE for your canal depth.
 *  4. Tune OPEN_GATE_THRESHOLD / CLOSE_GATE_THRESHOLD as needed.
 *  5. Flash to ESP8266 via Arduino IDE (Board: NodeMCU 1.0).
 *
 * ============================================================
 */


/* ============================================================
   SECTION 1 — BLYNK CREDENTIALS
   Replace every placeholder with your own Blynk project values.
   Never commit real tokens to a public repository — use a
   secrets file or environment variable in production.
   ============================================================ */

#define BLYNK_TEMPLATE_ID      "YOUR_TEMPLATE_ID"    // From Blynk console → Template → Template ID
#define BLYNK_TEMPLATE_NAME    "YourTemplateName"     // Human-readable template name (any string)
#define BLYNK_AUTH_TOKEN       "YOUR_AUTH_TOKEN"      // From Blynk console → Device → Auth Token

// Route Blynk debug output to the hardware Serial port so you
// can watch connection events in the Arduino Serial Monitor.
#define BLYNK_PRINT Serial


/* ============================================================
   SECTION 2 — LIBRARY INCLUDES
   ============================================================ */

#include <ESP8266WiFi.h>          // Core WiFi driver for ESP8266
#include <BlynkSimpleEsp8266.h>   // Blynk client library for ESP8266


/* ============================================================
   SECTION 3 — WIFI CREDENTIALS
   ============================================================ */

char ssid[] = "YOUR_SSID";       // Your WiFi network name (2.4 GHz only on ESP8266)
char pass[] = "YOUR_PASSWORD";   // Your WiFi password


/* ============================================================
   SECTION 4 — PIN DEFINITIONS
   NodeMCU labels (D0–D8) are used here for readability.
   The comments show the underlying GPIO numbers for reference.
   ============================================================ */

// ── Ultrasonic Sensor 1 (monitors Gate 1 / left channel) ────
#define TRIG_PIN1   D7   // GPIO13 — sends the ultrasonic pulse
#define ECHO_PIN1   D0   // GPIO16 — receives the returning echo

// ── Ultrasonic Sensor 2 (monitors Gate 2 / right channel) ───
#define TRIG_PIN2   D2   // GPIO4  — sends the ultrasonic pulse
#define ECHO_PIN2   D1   // GPIO5  — receives the returning echo

// ── Motor Driver 1 — controls Gate 1 ────────────────────────
// IN1 HIGH + IN2 LOW  → motor spins FORWARD  (gate closes)
// IN1 LOW  + IN2 HIGH → motor spins BACKWARD (gate opens)
// IN1 LOW  + IN2 LOW  → motor STOPPED
#define MOTOR1_IN1  D4   // GPIO2  — H-bridge input 1 for Gate 1
#define MOTOR1_IN2  D3   // GPIO0  — H-bridge input 2 for Gate 1

// ── Motor Driver 2 — controls Gate 2 ────────────────────────
#define MOTOR2_IN1  D5   // GPIO14 — H-bridge input 1 for Gate 2
#define MOTOR2_IN2  D6   // GPIO12 — H-bridge input 2 for Gate 2


/* ============================================================
   SECTION 5 — SENSOR DISTANCE THRESHOLDS
   These define the physical measurement range of the HC-SR04
   as installed in your canal. Values are in MILLIMETRES.

   How to calibrate:
     • MIN_DISTANCE: measure the sensor-to-water distance when
       the canal is at MAXIMUM water level (sensor reads closest).
     • MAX_DISTANCE: measure when the canal is at MINIMUM level
       (sensor reads farthest / empty canal).

   Everything outside this range is clamped to prevent
   erroneous percentage readings.
   ============================================================ */

#define MIN_DISTANCE  24    // mm — sensor distance at 100% water level
#define MAX_DISTANCE  92    // mm — sensor distance at   0% water level


/* ============================================================
   SECTION 6 — GATE CONTROL THRESHOLDS (percentage-based)
   These are the water-level percentages that trigger gate
   movement. A hysteresis band between the two values prevents
   the motor from rapidly toggling when the level is borderline.

   Example with defaults:
     • Level drops below 50% → water too LOW  → open gate  (let more in)
     • Level rises above 70% → water too HIGH → close gate (stop inflow)
     • Level between 50–70%  → no action      (stable zone / hysteresis)
   ============================================================ */

#define OPEN_GATE_THRESHOLD   70   // % — close the gate above this level
#define CLOSE_GATE_THRESHOLD  50   // % — open  the gate below this level


/* ============================================================
   SECTION 7 — MOTOR TIMING CONSTANTS
   Fine-tune these to match your physical gate mechanism.
   ============================================================ */

// How long the motor runs each time it receives an open/close
// command. Increase if the gate does not travel far enough in
// one activation cycle; decrease if it overshoots.
#define MOTOR_MOVE_DURATION  1000   // milliseconds per actuation

// Minimum idle time between consecutive motor movements on the
// same gate. Prevents continuous hammering if the water level
// keeps crossing a threshold. Acts as a mechanical debounce.
#define MOTOR_REST_DURATION  2000   // milliseconds between moves


/* ============================================================
   SECTION 8 — BLYNK VIRTUAL PIN ALIASES
   Centralised in one place so any pin reassignment only
   requires a change here, not throughout the code.
   ============================================================ */

#define VIRTUAL_PIN_SENSOR1    V0   // Gauge: Sensor 1 level (%)
#define VIRTUAL_PIN_SENSOR2    V1   // Gauge: Sensor 2 level (%)
#define VIRTUAL_PIN_AUTOMATION V2   // Toggle: Automation ON/OFF
#define VIRTUAL_PIN_GATE1      V3   // Button: Gate 1 status / manual override
#define VIRTUAL_PIN_GATE2      V4   // Button: Gate 2 status / manual override


/* ============================================================
   SECTION 9 — GLOBAL STATE VARIABLES
   ============================================================ */

/**
 * MotorDirection enum
 * Tracks the last direction each motor moved so the firmware
 * can avoid issuing a redundant command in the same direction,
 * and to enforce the rest period before reversing.
 */
enum MotorDirection {
  NONE,      // Motor has not moved since boot (initial state)
  FORWARD,   // Motor last ran forward  → gate was CLOSING
  BACKWARD   // Motor last ran backward → gate was OPENING
};

// ── Per-gate timing trackers ─────────────────────────────────
// Stores the millis() timestamp of the last motor activation.
// Used to enforce MOTOR_REST_DURATION between movements.
unsigned long lastMovementTime1 = 0;   // Gate 1
unsigned long lastMovementTime2 = 0;   // Gate 2

// ── Per-gate direction memory ────────────────────────────────
// Remembers the last movement direction so redundant or
// immediately reversed commands can be filtered out.
MotorDirection lastDirection1 = NONE;  // Gate 1
MotorDirection lastDirection2 = NONE;  // Gate 2

// ── System mode ──────────────────────────────────────────────
// true  → automation is active; sensors drive the gates.
// false → manual mode; gates are controlled from Blynk buttons.
bool automationEnabled = false;

// ── Manual gate states ───────────────────────────────────────
// Tracks the desired state of each gate when in manual mode.
// true = open, false = closed.
bool manualGate1State = false;
bool manualGate2State = false;


/* ============================================================
   SECTION 10 — FUNCTION FORWARD DECLARATIONS
   Allows the compiler to resolve function calls before their
   full definitions appear later in the file.
   ============================================================ */

void handleSensorAndMotor(int trigPin, int echoPin, int virtualPin,
                          unsigned long &lastMoveTime, MotorDirection &lastDir,
                          int motorIn1, int motorIn2, unsigned long currentTime,
                          bool invertLogic, int virtualGatePin);
void controlGate(bool open, int motorIn1, int motorIn2);
long getDistance(int trigPin, int echoPin);
int  clampDistance(int distance);
void moveMotorForward(int in1, int in2);
void moveMotorBackward(int in1, int in2);
void stopMotor(int in1, int in2);


/* ============================================================
   SECTION 11 — SETUP
   Runs once on power-on or after a reset.
   ============================================================ */

void setup() {

  // Start Serial at 115200 baud for debug output.
  // Open Arduino IDE → Tools → Serial Monitor at the same baud.
  Serial.begin(115200);
  Serial.println("\n[BOOT] Canal Gate Automation System starting...");

  // ── Connect to Blynk ───────────────────────────────────────
  // This call handles WiFi association AND Blynk server
  // connection. It blocks until both succeed.
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("[BOOT] Blynk connected.");

  // ── Configure Ultrasonic Sensor Pins ──────────────────────
  // TRIG pins drive the 10µs pulse output.
  // ECHO pins receive the reflected pulse input.
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
  Serial.println("[BOOT] Ultrasonic sensor pins configured.");

  // ── Configure Motor Driver Pins ───────────────────────────
  // All four control lines default LOW (motors stopped).
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);

  // Explicitly stop both motors at boot to prevent a runaway
  // condition if the H-bridge latched a previous state.
  stopMotor(MOTOR1_IN1, MOTOR1_IN2);
  stopMotor(MOTOR2_IN1, MOTOR2_IN2);
  Serial.println("[BOOT] Motor pins configured. Both gates stopped.");

  // ── Sync Blynk App Button States ──────────────────────────
  // Push the initial gate states to the app so the buttons
  // reflect reality from the moment the device comes online.
  Blynk.virtualWrite(VIRTUAL_PIN_GATE1, manualGate1State ? 1 : 0);
  Blynk.virtualWrite(VIRTUAL_PIN_GATE2, manualGate2State ? 1 : 0);
  Serial.println("[BOOT] Blynk virtual pins synced. System ready.\n");
}


/* ============================================================
   SECTION 12 — BLYNK CALLBACK HANDLERS
   These functions are called automatically by the Blynk library
   whenever the mobile app sends a new value on the matching
   virtual pin. They must follow the BLYNK_WRITE(Vx) signature.
   ============================================================ */

/**
 * VIRTUAL_PIN_AUTOMATION (V2) — Automation toggle
 *
 * Fired when the user taps the ON/OFF button in the Blynk app.
 *   param = 1 → Automation ON  (sensors control the gates)
 *   param = 0 → Automation OFF (manual button control active)
 */
BLYNK_WRITE(VIRTUAL_PIN_AUTOMATION) {
  automationEnabled = param.asInt();   // 0 or 1
  Serial.print("[MODE] Automation ");
  Serial.println(automationEnabled ? "ENABLED — sensors are in control."
                                   : "DISABLED — manual mode active.");
}

/**
 * VIRTUAL_PIN_GATE1 (V3) — Gate 1 manual toggle
 *
 * Only acts when automation is OFF. If automation is ON, the
 * button state is overwritten by the sensor logic anyway, so
 * accepting manual commands would cause a conflict.
 *
 *   param = 1 → user wants Gate 1 OPEN
 *   param = 0 → user wants Gate 1 CLOSED
 */
BLYNK_WRITE(VIRTUAL_PIN_GATE1) {
  if (!automationEnabled) {
    manualGate1State = param.asInt();
    Serial.print("[MANUAL] Gate 1 command received: ");
    Serial.println(manualGate1State ? "OPEN" : "CLOSE");
    controlGate(manualGate1State, MOTOR1_IN1, MOTOR1_IN2);
  } else {
    Serial.println("[MANUAL] Gate 1 command ignored — automation is active.");
  }
}

/**
 * VIRTUAL_PIN_GATE2 (V4) — Gate 2 manual toggle
 *
 * Identical logic to Gate 1 handler above, applied to Gate 2.
 */
BLYNK_WRITE(VIRTUAL_PIN_GATE2) {
  if (!automationEnabled) {
    manualGate2State = param.asInt();
    Serial.print("[MANUAL] Gate 2 command received: ");
    Serial.println(manualGate2State ? "OPEN" : "CLOSE");
    controlGate(manualGate2State, MOTOR2_IN1, MOTOR2_IN2);
  } else {
    Serial.println("[MANUAL] Gate 2 command ignored — automation is active.");
  }
}


/* ============================================================
   SECTION 13 — MAIN LOOP
   Runs continuously after setup(). Blynk.run() must be called
   every iteration to keep the WiFi connection alive and process
   any incoming app commands.
   ============================================================ */

void loop() {

  // Process Blynk communication and fire any pending callbacks.
  // Never block for long periods without calling this or the
  // connection will drop.
  Blynk.run();

  // Capture the current timestamp once per loop. Both sensor
  // handlers use this same value so timing is consistent.
  unsigned long currentTime = millis();

  // ── Gate 1: Sensor → Decision → Motor ─────────────────────
  // invertLogic = true because sensors mount above water (closer
  // reading = higher water = higher percentage).
  handleSensorAndMotor(
    TRIG_PIN1, ECHO_PIN1,          // Sensor 1 pins
    VIRTUAL_PIN_SENSOR1,           // Blynk gauge pin
    lastMovementTime1,             // Timing reference (passed by ref)
    lastDirection1,                // Direction memory (passed by ref)
    MOTOR1_IN1, MOTOR1_IN2,        // Motor 1 driver pins
    currentTime,                   // Current timestamp
    true,                          // invertLogic — sensor mounted above water
    VIRTUAL_PIN_GATE1              // Blynk gate button to reflect status
  );

  // ── Gate 2: Sensor → Decision → Motor ─────────────────────
  handleSensorAndMotor(
    TRIG_PIN2, ECHO_PIN2,
    VIRTUAL_PIN_SENSOR2,
    lastMovementTime2,
    lastDirection2,
    MOTOR2_IN1, MOTOR2_IN2,
    currentTime,
    true,
    VIRTUAL_PIN_GATE2
  );

  // 100 ms pause gives the ESP8266 background tasks (WiFi stack,
  // watchdog timer) time to run, and limits sensor polling to
  // ~10 readings per second — more than sufficient for a canal.
  delay(100);
}


/* ============================================================
   SECTION 14 — CORE SENSOR + MOTOR LOGIC
   ============================================================ */

/**
 * handleSensorAndMotor()
 * ──────────────────────
 * The heart of the automation engine. Called once per loop
 * iteration for each gate. Performs three jobs:
 *
 *   1. READ   — fires the ultrasonic sensor and converts the
 *               echo duration to a water-level percentage.
 *   2. REPORT — sends the percentage to the Blynk app gauge
 *               and updates the gate button state.
 *   3. ACT    — if automation is enabled and enough rest time
 *               has elapsed, drives the motor to open or close
 *               the gate based on the current water level.
 *
 * @param trigPin       GPIO pin connected to HC-SR04 TRIG
 * @param echoPin       GPIO pin connected to HC-SR04 ECHO
 * @param virtualPin    Blynk V-pin for the level gauge widget
 * @param lastMoveTime  Reference — millis() of last motor move
 * @param lastDir       Reference — direction of last motor move
 * @param motorIn1      H-bridge IN1 pin for this gate's motor
 * @param motorIn2      H-bridge IN2 pin for this gate's motor
 * @param currentTime   millis() snapshot from this loop tick
 * @param invertLogic   true  = sensor above water (standard mounting)
 *                      false = sensor below water (inverted mounting)
 * @param virtualGatePin Blynk V-pin for the gate status button
 */
void handleSensorAndMotor(int trigPin, int echoPin, int virtualPin,
                          unsigned long &lastMoveTime, MotorDirection &lastDir,
                          int motorIn1, int motorIn2, unsigned long currentTime,
                          bool invertLogic, int virtualGatePin) {

  // ── Step 1: Measure distance ────────────────────────────────
  // getDistance() returns the raw echo duration in microseconds.
  // Speed of sound ≈ 0.343 mm/µs. Divide by 2 because the pulse
  // travels to the water surface AND back.
  long duration    = getDistance(trigPin, echoPin);
  int  distance_mm = clampDistance((int)(duration * 0.343 / 2.0));

  // ── Step 2: Convert distance to water level percentage ─────
  // map() performs a linear interpolation:
  //   MIN_DISTANCE → 100% (sensor close to water = full canal)
  //   MAX_DISTANCE →   0% (sensor far from water = empty canal)
  // This inversion corrects for the downward-facing mounting.
  int sensorPercentage = map(distance_mm, MIN_DISTANCE, MAX_DISTANCE, 100, 0);

  // ── Step 3: Push level reading to Blynk app ────────────────
  Blynk.virtualWrite(virtualPin, sensorPercentage);

  // Print to Serial Monitor for on-bench debugging.
  Serial.print("[SENSOR] Distance: ");
  Serial.print(distance_mm);
  Serial.print(" mm  →  Level: ");
  Serial.print(sensorPercentage);
  Serial.println(" %");

  // ── Step 4: Update gate button state in app ────────────────
  // In auto mode the button reflects the sensor-driven decision.
  // In manual mode we leave the button alone (user controls it).
  if (automationEnabled) {
    // Determine whether the gate should be open right now.
    // invertLogic = true:  open when level is LOW (< CLOSE threshold)
    // invertLogic = false: open when level is HIGH (> OPEN threshold)
    bool gateOpen = invertLogic
                      ? (sensorPercentage < CLOSE_GATE_THRESHOLD)
                      : (sensorPercentage > OPEN_GATE_THRESHOLD);
    Blynk.virtualWrite(virtualGatePin, gateOpen ? 1 : 0);
  } else {
    // Manual mode — nothing to automate, exit early.
    return;
  }

  // ── Step 5: Evaluate gate open/close conditions ────────────
  //
  // openGateCondition  — true when water is too LOW → gate should open
  // closeGateCondition — true when water is too HIGH → gate should close
  //
  // Note: only ONE of these will be true at a time because of
  // the hysteresis band between CLOSE_GATE_THRESHOLD (50%) and
  // OPEN_GATE_THRESHOLD (70%). When the level is between 50–70%
  // neither condition fires and the motor stays still.
  bool openGateCondition  = invertLogic
                              ? (sensorPercentage < CLOSE_GATE_THRESHOLD)
                              : (sensorPercentage > OPEN_GATE_THRESHOLD);

  bool closeGateCondition = invertLogic
                              ? (sensorPercentage > OPEN_GATE_THRESHOLD)
                              : (sensorPercentage < CLOSE_GATE_THRESHOLD);

  // ── Step 6: Execute motor commands ─────────────────────────
  //
  // Guards before any motor activation:
  //   (a) The condition must be true (level out of bounds).
  //   (b) Enough rest time must have elapsed since last move
  //       (prevents continuous hammering on a borderline reading).
  //   (c) The motor must not already be at the target state
  //       (lastDir check prevents re-issuing the same command).

  if (openGateCondition &&
      (currentTime - lastMoveTime) > MOTOR_REST_DURATION &&
      lastDir != BACKWARD) {

    Serial.println("[MOTOR] Water LOW — opening gate...");
    moveMotorBackward(motorIn1, motorIn2);   // BACKWARD = gate rises (open)
    delay(MOTOR_MOVE_DURATION);             // Hold for configured duration
    stopMotor(motorIn1, motorIn2);          // Cut power — gate holds position
    lastMoveTime = millis();               // Record activation timestamp
    lastDir      = BACKWARD;              // Remember direction for next check

  } else if (closeGateCondition &&
             (currentTime - lastMoveTime) > MOTOR_REST_DURATION &&
             lastDir != FORWARD) {

    Serial.println("[MOTOR] Water HIGH — closing gate...");
    moveMotorForward(motorIn1, motorIn2);    // FORWARD = gate lowers (close)
    delay(MOTOR_MOVE_DURATION);
    stopMotor(motorIn1, motorIn2);
    lastMoveTime = millis();
    lastDir      = FORWARD;
  }
  // If neither condition fires, the motor remains stopped and
  // the gate holds its last mechanical position.
}


/* ============================================================
   SECTION 15 — MANUAL GATE CONTROL
   ============================================================ */

/**
 * controlGate()
 * ─────────────
 * Drives a gate motor in response to a manual command from the
 * Blynk app (used only when automation is disabled).
 *
 * The motor runs for exactly MOTOR_MOVE_DURATION milliseconds
 * then stops. The gate's physical position is maintained by the
 * lead screw mechanism — no holding current is required.
 *
 * @param open     true  → run motor backward (raise gate / open)
 *                 false → run motor forward  (lower gate / close)
 * @param motorIn1 H-bridge IN1 pin
 * @param motorIn2 H-bridge IN2 pin
 */
void controlGate(bool open, int motorIn1, int motorIn2) {
  if (open) {
    moveMotorBackward(motorIn1, motorIn2);  // Raise gate
  } else {
    moveMotorForward(motorIn1, motorIn2);   // Lower gate
  }
  delay(MOTOR_MOVE_DURATION);  // Allow mechanism to travel
  stopMotor(motorIn1, motorIn2);           // Stop and hold
}


/* ============================================================
   SECTION 16 — ULTRASONIC SENSOR FUNCTIONS
   ============================================================ */

/**
 * getDistance()
 * ─────────────
 * Triggers a single HC-SR04 measurement cycle and returns the
 * echo pulse duration in microseconds.
 *
 * HC-SR04 measurement sequence:
 *   1. Pull TRIG LOW for 2 µs to clear any residual signal.
 *   2. Pull TRIG HIGH for exactly 10 µs — this fires 8 ultrasonic
 *      bursts at 40 kHz.
 *   3. Pull TRIG LOW to end the trigger pulse.
 *   4. pulseIn() waits for ECHO to go HIGH, then measures how
 *      long it stays HIGH (= round-trip travel time of the burst).
 *
 * Conversion to distance:
 *   distance_mm = duration_µs × 0.343 mm/µs ÷ 2
 *
 * @param trigPin  GPIO connected to HC-SR04 TRIG
 * @param echoPin  GPIO connected to HC-SR04 ECHO
 * @return         Echo pulse duration in microseconds
 */
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);          // Ensure TRIG starts clean

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);         // 10 µs trigger pulse (HC-SR04 spec)
  digitalWrite(trigPin, LOW);

  // pulseIn() returns 0 if the echo times out (> 1 second by default).
  // A 0 return will be clamped to MIN_DISTANCE by clampDistance().
  return pulseIn(echoPin, HIGH);
}

/**
 * clampDistance()
 * ───────────────
 * Constrains a raw distance value to the valid operating range
 * defined by MIN_DISTANCE and MAX_DISTANCE.
 *
 * Out-of-range readings can occur when:
 *   • The sensor detects air bubbles or foam on the water surface.
 *   • pulseIn() times out and returns 0.
 *   • An object other than water reflects the pulse.
 *
 * Clamping prevents the map() call from producing percentages
 * outside the 0–100% range.
 *
 * @param distance Raw distance in millimetres
 * @return         Clamped distance within [MIN_DISTANCE, MAX_DISTANCE]
 */
int clampDistance(int distance) {
  if (distance < MIN_DISTANCE) return MIN_DISTANCE;
  if (distance > MAX_DISTANCE) return MAX_DISTANCE;
  return distance;
}


/* ============================================================
   SECTION 17 — MOTOR DRIVER PRIMITIVES
   These three functions are the only interface to the L298N
   H-bridge. All higher-level logic calls these rather than
   writing to motor pins directly.

   L298N truth table (per motor channel):
   ┌─────┬─────┬───────────────────┐
   │ IN1 │ IN2 │ Motor behaviour   │
   ├─────┼─────┼───────────────────┤
   │  H  │  L  │ Forward (close)   │
   │  L  │  H  │ Backward (open)   │
   │  L  │  L  │ Stopped (coast)   │
   │  H  │  H  │ Brake (avoid!)    │
   └─────┴─────┴───────────────────┘
   ============================================================ */

/**
 * moveMotorForward() — Lowers the gate (closes the water flow).
 * IN1 HIGH, IN2 LOW → motor spins in the closing direction.
 */
void moveMotorForward(int in1, int in2) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

/**
 * moveMotorBackward() — Raises the gate (opens the water flow).
 * IN1 LOW, IN2 HIGH → motor spins in the opening direction.
 */
void moveMotorBackward(int in1, int in2) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

/**
 * stopMotor() — Cuts power to the motor (coast stop).
 * IN1 LOW, IN2 LOW → both H-bridge inputs off → motor coasts to rest.
 * The lead-screw mechanism holds the gate position mechanically
 * so no active braking or holding current is needed.
 */
void stopMotor(int in1, int in2) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}
