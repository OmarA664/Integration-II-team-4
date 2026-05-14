// ============================================================
//  LINE-FOLLOWING ROBOT  —  Arduino Nano
//  Hardware: 4 DC motors, 2 IR sensors, 1 TCS3200 color sensor
// ============================================================
//
//  LIBRARY REQUIRED:
//    None.  The TCS3200 color sensor is driven with raw
//    pulseIn() calls — no extra library needed.  The sensor
//    outputs a square wave whose frequency is proportional to
//    detected light; all reading logic is in readColorSensor().
//
//  GREEN LINE — START / END LOGIC:
//    Green is both the start and end line.
//    At power-on the robot sits on the green start line.
//    A "hasLeftStartLine" flag prevents it from immediately
//    re-detecting green as the finish.  The flag flips to true
//    once the robot leaves green (sees black floor or another
//    color) AND a minimum travel delay has elapsed.
//    After that, the next green detection stops the robot.
//
//  ORANGE INTERSECTION:
//    Orange means a multi-color intersection.
//    The robot always drives straight through orange.
//
// ============================================================


// ============================================================
//  SECTION 1 — PIN ASSIGNMENTS
//  Change these numbers to match your actual wiring.
//  All other code uses these names — only edit here.
// ============================================================

// --- IR Line Sensors (digital: LOW = on line, HIGH = off line)
const int PIN_IR_LEFT  = 2;   // Left  IR sensor OUT pin
const int PIN_IR_RIGHT = 3;   // Right IR sensor OUT pin

// --- TCS3200 Color Sensor
const int PIN_COLOR_S0  = 4;  // Frequency scaling S0
const int PIN_COLOR_S1  = 5;  // Frequency scaling S1
const int PIN_COLOR_S2  = 6;  // Filter select S2
const int PIN_COLOR_S3  = 7;  // Filter select S3
const int PIN_COLOR_OUT = 8;  // Frequency output from sensor

// --- Motor Driver (4 direction pins + 4 PWM speed pins)
//     Motor layout (top-down view):
//       Front-Left (FL)   Front-Right (FR)
//       Rear-Left  (RL)   Rear-Right  (RR)

const int PIN_FL_IN1 = 9;    // Front-Left  direction A
const int PIN_FL_IN2 = 10;   // Front-Left  direction B
const int PIN_FR_IN1 = 11;   // Front-Right direction A
const int PIN_FR_IN2 = 12;   // Front-Right direction B
const int PIN_RL_IN1 = A0;   // Rear-Left   direction A
const int PIN_RL_IN2 = A1;   // Rear-Left   direction B
const int PIN_RR_IN1 = A2;   // Rear-Right  direction A
const int PIN_RR_IN2 = A3;   // Rear-Right  direction B

// PWM enable pins (connect to ENA/ENB on L298N).
// If your board ties these HIGH permanently, you can remove
// the analogWrite() calls in the movement functions below.
const int PIN_FL_EN = 5;     // Front-Left  PWM  (must be PWM pin)
const int PIN_FR_EN = 6;     // Front-Right PWM  (must be PWM pin)
const int PIN_RL_EN = 9;     // Rear-Left   PWM  (must be PWM pin)
const int PIN_RR_EN = 10;    // Rear-Right  PWM  (must be PWM pin)

// NOTE: Nano PWM-capable pins are 3, 5, 6, 9, 10, 11.
//       Make sure all EN pins land on one of those.


// ============================================================
//  SECTION 2 — SPEED SETTINGS
//  Range 0 (stopped) to 255 (full speed).
//  Start low (~120) while testing so the robot is controllable.
// ============================================================

const int SPEED_FORWARD = 150;  // Normal cruising speed
const int SPEED_TURN    = 120;  // Speed of the faster side when correcting
const int SPEED_SLOW    =  90;  // Speed of the slower side when correcting


// ============================================================
//  SECTION 3 — COLOR SENSOR CALIBRATION
//
//  HOW TO CALIBRATE:
//    1. Upload this sketch and open the Serial Monitor (9600).
//    2. Uncomment the Serial.print lines inside readColorSensor()
//       at the bottom of this file.
//    3. Hold the sensor ~1–2 cm above each colored surface.
//    4. Note the printed R, G, B values (pulse-width in µs).
//       Lower value = sensor detects more of that color.
//    5. Set each color's MIN and MAX to bracket those readings
//       with a little margin (e.g. observed=40, set MIN=25 MAX=55).
//    6. Re-comment the Serial.print lines when done to speed
//       up the main loop.
//
//  The floor is BLACK — it absorbs all light, so all three
//  channels will read very HIGH numbers.  No thresholds are
//  needed for black; it falls through as COLOR_UNKNOWN.
// ============================================================

// ---- GREEN line thresholds  (START line & END line) --------
//  Green reflects strongly in G channel; R and B stay high.
const int GREEN_R_MIN = 80;   const int GREEN_R_MAX = 200;
const int GREEN_G_MIN = 10;   const int GREEN_G_MAX = 60;
const int GREEN_B_MIN = 80;   const int GREEN_B_MAX = 200;

// ---- ORANGE line thresholds  (INTERSECTION — go straight) --
//  Orange: strong R (low µs), medium G, very weak B (high µs).
const int ORANGE_R_MIN = 10;  const int ORANGE_R_MAX = 50;
const int ORANGE_G_MIN = 40;  const int ORANGE_G_MAX = 120;
const int ORANGE_B_MIN = 120; const int ORANGE_B_MAX = 255;

// ---- RED line thresholds -----------------------------------
const int RED_R_MIN = 10;    const int RED_R_MAX = 60;
const int RED_G_MIN = 80;    const int RED_G_MAX = 200;
const int RED_B_MIN = 80;    const int RED_B_MAX = 200;

// ---- BLUE line thresholds ----------------------------------
const int BLUE_R_MIN = 80;   const int BLUE_R_MAX = 200;
const int BLUE_G_MIN = 80;   const int BLUE_G_MAX = 200;
const int BLUE_B_MIN = 10;   const int BLUE_B_MAX = 60;

// ---- BLACK floor threshold ---------------------------------
//  All three channels read >= this value on black.
//  Raise this number if the robot misidentifies dark surfaces.
const int BLACK_CHANNEL_MIN = 250;


// ============================================================
//  SECTION 4 — START / END LINE TIMING
//
//  START_LEAVE_DELAY_MS:
//    Minimum milliseconds after startup before the robot will
//    ever treat green as the "end line".
//    This is a safety net — even if the sensor somehow still
//    sees green after physically leaving the start, the robot
//    won't stop until this delay has elapsed AND it has seen
//    a non-green reading at least once.
//    2000 ms (2 seconds) is a safe default.  Increase it if
//    your start line is wide and the robot crosses it slowly.
// ============================================================

const unsigned long START_LEAVE_DELAY_MS = 2000;


// ============================================================
//  SECTION 5 — COLOR ENUM
//  Symbolic names used throughout the sketch.
// ============================================================

enum LineColor {
  COLOR_UNKNOWN = 0,  // Black floor or unrecognized surface
  COLOR_GREEN,        // Start line / End line
  COLOR_ORANGE,       // Intersection — always go straight
  COLOR_RED,
  COLOR_BLUE
};


// ============================================================
//  SECTION 6 — STATE VARIABLES
//  Managed automatically — do not edit these values.
// ============================================================

// Becomes true once the robot has physically driven off the
// green start line.  Until then, green detections are ignored.
bool hasLeftStartLine = false;

// Timestamp (ms) set at the end of setup().
// Compared against millis() to enforce the minimum run time.
unsigned long startTime = 0;

// Set to true when the robot stops at the end line.
// The main loop checks this and keeps motors off permanently.
bool courseFinished = false;


// ============================================================
//  SECTION 7 — IR SENSOR CONVENTION
//  Swap the values here if your sensors output inverted logic.
// ============================================================

const int ON_LINE  = LOW;   // Sensor output when over the line
const int OFF_LINE = HIGH;  // Sensor output when off the line


// ============================================================
//  SECTION 8 — SETUP
// ============================================================

void setup() {
  Serial.begin(9600);

  // IR sensor pins — inputs
  pinMode(PIN_IR_LEFT,  INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);

  // Color sensor control pins — outputs
  pinMode(PIN_COLOR_S0,  OUTPUT);
  pinMode(PIN_COLOR_S1,  OUTPUT);
  pinMode(PIN_COLOR_S2,  OUTPUT);
  pinMode(PIN_COLOR_S3,  OUTPUT);
  pinMode(PIN_COLOR_OUT, INPUT);

  // TCS3200 output frequency scaling.
  // S0=HIGH, S1=LOW → 20% scaling (good for Nano's pulseIn).
  // To use 2%:   S0=LOW,  S1=HIGH
  // To use 100%: S0=HIGH, S1=HIGH
  digitalWrite(PIN_COLOR_S0, HIGH);
  digitalWrite(PIN_COLOR_S1, LOW);

  // Motor direction pins — outputs
  pinMode(PIN_FL_IN1, OUTPUT); pinMode(PIN_FL_IN2, OUTPUT);
  pinMode(PIN_FR_IN1, OUTPUT); pinMode(PIN_FR_IN2, OUTPUT);
  pinMode(PIN_RL_IN1, OUTPUT); pinMode(PIN_RL_IN2, OUTPUT);
  pinMode(PIN_RR_IN1, OUTPUT); pinMode(PIN_RR_IN2, OUTPUT);

  // Motor enable (PWM) pins — outputs
  pinMode(PIN_FL_EN, OUTPUT); pinMode(PIN_FR_EN, OUTPUT);
  pinMode(PIN_RL_EN, OUTPUT); pinMode(PIN_RR_EN, OUTPUT);

  stopMotors();  // Ensure the robot is completely still at boot

  // Record the exact time setup finishes.
  // Used to enforce START_LEAVE_DELAY_MS before the end-line
  // detector becomes active.
  startTime = millis();

  Serial.println("=================================");
  Serial.println("  Line Follower — Ready");
  Serial.println("  Sitting on GREEN start line.");
  Serial.println("  Starting in 2 seconds...");
  Serial.println("=================================");
  delay(2000);  // Brief pause so you can stand clear
}


// ============================================================
//  SECTION 9 — MAIN LOOP
// ============================================================

void loop() {
  // Once the course is finished, keep motors off indefinitely.
  // Reset the Arduino to run the course again.
  if (courseFinished) {
    stopMotors();
    return;
  }

  followLine();
}


// ============================================================
//  SECTION 10 — MOVEMENT FUNCTIONS
// ============================================================

// Internal helper: set one motor's direction and speed.
void setMotor(int in1, int in2, int enPin, int spd, bool fwd) {
  analogWrite(enPin, spd);
  if (fwd) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
}

// Drive all four wheels forward at SPEED_FORWARD.
void moveForward() {
  setMotor(PIN_FL_IN1, PIN_FL_IN2, PIN_FL_EN, SPEED_FORWARD, true);
  setMotor(PIN_FR_IN1, PIN_FR_IN2, PIN_FR_EN, SPEED_FORWARD, true);
  setMotor(PIN_RL_IN1, PIN_RL_IN2, PIN_RL_EN, SPEED_FORWARD, true);
  setMotor(PIN_RR_IN1, PIN_RR_IN2, PIN_RR_EN, SPEED_FORWARD, true);
}

// Curve left: slow left wheels, fast right wheels.
// The robot bends left while still moving forward.
void turnLeft() {
  setMotor(PIN_FL_IN1, PIN_FL_IN2, PIN_FL_EN, SPEED_SLOW, true);
  setMotor(PIN_FR_IN1, PIN_FR_IN2, PIN_FR_EN, SPEED_TURN, true);
  setMotor(PIN_RL_IN1, PIN_RL_IN2, PIN_RL_EN, SPEED_SLOW, true);
  setMotor(PIN_RR_IN1, PIN_RR_IN2, PIN_RR_EN, SPEED_TURN, true);
}

// Curve right: slow right wheels, fast left wheels.
void turnRight() {
  setMotor(PIN_FL_IN1, PIN_FL_IN2, PIN_FL_EN, SPEED_TURN, true);
  setMotor(PIN_FR_IN1, PIN_FR_IN2, PIN_FR_EN, SPEED_SLOW, true);
  setMotor(PIN_RL_IN1, PIN_RL_IN2, PIN_RL_EN, SPEED_TURN, true);
  setMotor(PIN_RR_IN1, PIN_RR_IN2, PIN_RR_EN, SPEED_SLOW, true);
}

// Cut power to all motors immediately.
void stopMotors() {
  analogWrite(PIN_FL_EN, 0); analogWrite(PIN_FR_EN, 0);
  analogWrite(PIN_RL_EN, 0); analogWrite(PIN_RR_EN, 0);
  digitalWrite(PIN_FL_IN1, LOW); digitalWrite(PIN_FL_IN2, LOW);
  digitalWrite(PIN_FR_IN1, LOW); digitalWrite(PIN_FR_IN2, LOW);
  digitalWrite(PIN_RL_IN1, LOW); digitalWrite(PIN_RL_IN2, LOW);
  digitalWrite(PIN_RR_IN1, LOW); digitalWrite(PIN_RR_IN2, LOW);
}


// ============================================================
//  SECTION 11 — COLOR SENSOR FUNCTIONS
// ============================================================

// Read one channel of the TCS3200.
//   The sensor outputs a square wave.  pulseIn() measures how
//   long (µs) each LOW pulse lasts.  Shorter = more of that
//   color present.  Timeout = 100 ms (returns 0 on timeout).
//
//   Filter truth table:
//     S2    S3    Channel measured
//     LOW   LOW   Red
//     LOW   HIGH  Blue
//     HIGH  LOW   Clear (no filter — unused here)
//     HIGH  HIGH  Green
int readChannel(int s2State, int s3State) {
  digitalWrite(PIN_COLOR_S2, s2State);
  digitalWrite(PIN_COLOR_S3, s3State);
  delay(10);  // Let sensor settle after switching filter
  return (int) pulseIn(PIN_COLOR_OUT, LOW, 100000);
}

// Read fresh R, G, B values from the color sensor.
// Results are stored in the three variables passed by reference.
void readColorSensor(int &r, int &g, int &b) {
  r = readChannel(LOW,  LOW);   // Red channel
  g = readChannel(HIGH, HIGH);  // Green channel
  b = readChannel(LOW,  HIGH);  // Blue channel

  // ---- CALIBRATION DEBUG OUTPUT ----
  // Uncomment these lines while calibrating; re-comment after.
  // Serial.print("R="); Serial.print(r);
  // Serial.print(" G="); Serial.print(g);
  // Serial.print(" B="); Serial.println(b);
}

// Classify an R,G,B reading into a LineColor value.
// ORANGE is checked first — it shares red/green characteristics
// and must be caught before the individual color checks.
LineColor detectLineColor(int r, int g, int b) {

  // BLACK floor: all channels return very high values (no reflection).
  if (r >= BLACK_CHANNEL_MIN &&
      g >= BLACK_CHANNEL_MIN &&
      b >= BLACK_CHANNEL_MIN) {
    return COLOR_UNKNOWN;
  }

  // ORANGE — intersection signal, checked first for priority.
  if (r >= ORANGE_R_MIN && r <= ORANGE_R_MAX &&
      g >= ORANGE_G_MIN && g <= ORANGE_G_MAX &&
      b >= ORANGE_B_MIN && b <= ORANGE_B_MAX) {
    return COLOR_ORANGE;
  }

  // GREEN — start / end line.
  if (r >= GREEN_R_MIN && r <= GREEN_R_MAX &&
      g >= GREEN_G_MIN && g <= GREEN_G_MAX &&
      b >= GREEN_B_MIN && b <= GREEN_B_MAX) {
    return COLOR_GREEN;
  }

  // RED
  if (r >= RED_R_MIN && r <= RED_R_MAX &&
      g >= RED_G_MIN && g <= RED_G_MAX &&
      b >= RED_B_MIN && b <= RED_B_MAX) {
    return COLOR_RED;
  }

  // BLUE
  if (r >= BLUE_R_MIN && r <= BLUE_R_MAX &&
      g >= BLUE_G_MIN && g <= BLUE_G_MAX &&
      b >= BLUE_B_MIN && b <= BLUE_B_MAX) {
    return COLOR_BLUE;
  }

  return COLOR_UNKNOWN;
}


// ============================================================
//  SECTION 12 — START LINE DEPARTURE TRACKER
//
//  Call this every loop BEFORE acting on a green detection.
//
//  State machine:
//
//  [hasLeftStartLine = false]  ← initial state at boot
//    Robot is still on or very near the start line.
//    Green readings are silently ignored as "still at start".
//    The flag flips to TRUE only when BOTH are true:
//      (a) millis() - startTime  >=  START_LEAVE_DELAY_MS
//      (b) The current color is NOT green  (robot left the line)
//
//  [hasLeftStartLine = true]  ← active course-following state
//    Robot is out on the course.
//    The next green detection = end line → stop and finish.
// ============================================================

void updateStartLineState(LineColor currentColor) {
  if (!hasLeftStartLine) {
    bool delayElapsed = (millis() - startTime >= START_LEAVE_DELAY_MS);
    bool notOnGreen   = (currentColor != COLOR_GREEN);

    if (delayElapsed && notOnGreen) {
      hasLeftStartLine = true;
      Serial.println("--- Left start line. End-line detection is now ACTIVE. ---");
    }
  }
}


// ============================================================
//  SECTION 13 — LINE FOLLOWING LOGIC
//
//  Decision order each loop cycle:
//    1. Read IR sensors
//    2. Read and classify color sensor
//    3. Update start-line departure state
//    4. Handle GREEN (start ignore vs. end-line stop)
//    5. Handle ORANGE (intersection — go straight)
//    6. IR-based steering on normal path
// ============================================================

void followLine() {

  // --- 1. Read both IR sensors ---
  int leftSensor  = digitalRead(PIN_IR_LEFT);
  int rightSensor = digitalRead(PIN_IR_RIGHT);

  // --- 2. Read and classify the color sensor ---
  int r, g, b;
  readColorSensor(r, g, b);
  LineColor detectedColor = detectLineColor(r, g, b);

  // --- 3. Update start-line departure tracking ---
  //  Must run every cycle so the flag flips the moment the
  //  robot rolls clear of the start line.
  updateStartLineState(detectedColor);

  // --- 4. Handle GREEN ---
  if (detectedColor == COLOR_GREEN) {
    if (!hasLeftStartLine) {
      // Still near the start — drive forward and ignore.
      Serial.println("Green detected — still on start line, ignoring.");
      moveForward();
    } else {
      // Out on the course — this is the finish line.
      Serial.println(">>> END LINE REACHED — stopping. Course complete! <<<");
      stopMotors();
      courseFinished = true;
    }
    return;  // Do not execute IR steering below
  }

  // --- 5. Handle ORANGE intersection — always go straight ---
  if (detectedColor == COLOR_ORANGE) {
    Serial.println("Intersection (ORANGE) — proceeding straight.");
    moveForward();
    return;
  }

  // --- 6. Normal IR-based steering ---
  //
  //   Both ON    → centered          → full speed forward
  //   Left OFF   → drifted too left  → curve right to re-center
  //   Right OFF  → drifted too right → curve left  to re-center
  //   Both OFF   → line lost         → stop (add search here later)

  if (leftSensor == ON_LINE && rightSensor == ON_LINE) {
    moveForward();
  }
  else if (leftSensor == OFF_LINE && rightSensor == ON_LINE) {
    Serial.println("Drifted left — correcting right.");
    turnRight();
  }
  else if (leftSensor == ON_LINE && rightSensor == OFF_LINE) {
    Serial.println("Drifted right — correcting left.");
    turnLeft();
  }
  else {
    // Both sensors off the line.
    // Default: stop.  Replace with a spin-search routine if needed.
    Serial.println("Line lost — stopping.");
    stopMotors();
  }
}

// ============================================================
//  END OF SKETCH
// ============================================================
