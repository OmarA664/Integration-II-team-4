#include <Wire.h>
#include <Adafruit_TCS34725.h>

// ─────────────────────────────────────────────
// L298 Motor Controller Pin Assignment
// ─────────────────────────────────────────────

// Motor A - Front Left
const int enAF = 10;
const int in1F = 2;
const int in2F = 3;

// Motor B - Front Right
const int enBF = 11;
const int in3F = 4;
const int in4F = 5;

// Motor C - Back Left
const int enAB = 12;
const int in1B = 6;
const int in2B = 7;

// Motor D - Back Right
const int enBB = 13;
const int in3B = 8;
const int in4B = 9;

// ─────────────────────────────────────────────
// TCS34725 Color Sensor (I2C: SDA=20, SCL=21 on Mega)
// ─────────────────────────────────────────────

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ─────────────────────────────────────────────
// Line Following Settings
// ─────────────────────────────────────────────

const int DRIVE_SPEED   = 150;  // Base forward speed  (0–255)
const int TURN_SPEED    = 120;  // Speed used when correcting (0–255)

// What color is your line? Set one of these to true.
// Adjust the thresholds below after calibrating your sensor.
const bool FOLLOW_RED   = true;
const bool FOLLOW_BLUE  = false;
const bool FOLLOW_GREEN = false;

// Color detection thresholds — tune these for your environment/lighting
// Run the calibration helper in Serial Monitor to find good values
const int COLOR_THRESHOLD = 80;   // Minimum dominant channel value to confirm a color
const int CLEAR_THRESHOLD = 200;  // Minimum 'clear' reading to trust sensor (not in darkness)

// ─────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(enAF, OUTPUT);  pinMode(enBF, OUTPUT);
  pinMode(in1F, OUTPUT);  pinMode(in2F, OUTPUT);
  pinMode(in3F, OUTPUT);  pinMode(in4F, OUTPUT);

  pinMode(enAB, OUTPUT);  pinMode(enBB, OUTPUT);
  pinMode(in1B, OUTPUT);  pinMode(in2B, OUTPUT);
  pinMode(in3B, OUTPUT);  pinMode(in4B, OUTPUT);

  stopMotors();

  // Color sensor init
  if (!tcs.begin()) {
    Serial.println("ERROR: TCS34725 not found. Check wiring.");
    while (1);  // Halt if sensor missing
  }
  Serial.println("Color sensor ready.");
  delay(500);
}

// ─────────────────────────────────────────────
// MAIN LOOP — Line Following Logic
// ─────────────────────────────────────────────

void loop() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // Print readings to Serial for calibration
  Serial.print("R:"); Serial.print(r);
  Serial.print(" G:"); Serial.print(g);
  Serial.print(" B:"); Serial.print(b);
  Serial.print(" C:"); Serial.println(c);

  // If sensor is too dark / not over anything, stop
  if (c < CLEAR_THRESHOLD) {
    stopMotors();
    return;
  }

  LineColor detected = detectColor(r, g, b);

  if (isTargetColor(detected)) {
    // On the line — drive forward
    driveForward(DRIVE_SPEED);
  } else {
    // Off the line — search/correct
    // Simple correction: spin in place to find the line
    // For a more advanced PID approach, see the comment block below
    turnRight(TURN_SPEED);
    delay(100);
    stopMotors();
    delay(50);
  }
}

// ─────────────────────────────────────────────
// COLOR DETECTION
// ─────────────────────────────────────────────

enum LineColor { COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_UNKNOWN };

LineColor detectColor(uint16_t r, uint16_t g, uint16_t b) {
  // Determine which channel dominates
  if (r > g && r > b && r > COLOR_THRESHOLD) return COLOR_RED;
  if (g > r && g > b && g > COLOR_THRESHOLD) return COLOR_GREEN;
  if (b > r && b > g && b > COLOR_THRESHOLD) return COLOR_BLUE;
  return COLOR_UNKNOWN;
}

bool isTargetColor(LineColor c) {
  if (FOLLOW_RED   && c == COLOR_RED)   return true;
  if (FOLLOW_GREEN && c == COLOR_GREEN) return true;
  if (FOLLOW_BLUE  && c == COLOR_BLUE)  return true;
  return false;
}

// ─────────────────────────────────────────────
// DRIVING FUNCTIONS  (Speed: PWM 0–255)
// ─────────────────────────────────────────────

void driveForward(int speed) {
  analogWrite(enAF, speed);  analogWrite(enBF, speed);
  analogWrite(enAB, speed);  analogWrite(enBB, speed);

  digitalWrite(in1F, LOW);   digitalWrite(in2F, HIGH);  // Front Left  - forward
  digitalWrite(in3F, LOW);   digitalWrite(in4F, HIGH);  // Front Right - forward
  digitalWrite(in1B, LOW);   digitalWrite(in2B, HIGH);  // Back Left   - forward
  digitalWrite(in3B, LOW);   digitalWrite(in4B, HIGH);  // Back Right  - forward
}

void driveBackward(int speed) {
  analogWrite(enAF, speed);  analogWrite(enBF, speed);
  analogWrite(enAB, speed);  analogWrite(enBB, speed);

  digitalWrite(in1F, HIGH);  digitalWrite(in2F, LOW);   // Front Left  - backward
  digitalWrite(in3F, HIGH);  digitalWrite(in4F, LOW);   // Front Right - backward
  digitalWrite(in1B, HIGH);  digitalWrite(in2B, LOW);   // Back Left   - backward
  digitalWrite(in3B, HIGH);  digitalWrite(in4B, LOW);   // Back Right  - backward
}

void turnRight(int speed) {
  // Left side forward, right side backward → spins right
  analogWrite(enAF, speed);  analogWrite(enBF, speed);
  analogWrite(enAB, speed);  analogWrite(enBB, speed);

  digitalWrite(in1F, HIGH);  digitalWrite(in2F, LOW);   // Front Left  - forward
  digitalWrite(in3F, LOW);   digitalWrite(in4F, HIGH);  // Front Right - backward
  digitalWrite(in1B, LOW);   digitalWrite(in2B, HIGH);  // Back Left   - forward
  digitalWrite(in3B, HIGH);  digitalWrite(in4B, LOW);   // Back Right  - backward
}

void turnLeft(int speed) {
  // Right side forward, left side backward → spins left
  analogWrite(enAF, speed);  analogWrite(enBF, speed);
  analogWrite(enAB, speed);  analogWrite(enBB, speed);

  digitalWrite(in1F, LOW);   digitalWrite(in2F, HIGH);  // Front Left  - backward
  digitalWrite(in3F, HIGH);  digitalWrite(in4F, LOW);   // Front Right - forward
  digitalWrite(in1B, HIGH);  digitalWrite(in2B, LOW);   // Back Left   - backward
  digitalWrite(in3B, LOW);   digitalWrite(in4B, HIGH);  // Back Right  - forward
}

void stopMotors() {
  analogWrite(enAF, 0);  analogWrite(enBF, 0);   // Also zero PWM enables
  analogWrite(enAB, 0);  analogWrite(enBB, 0);

  digitalWrite(in1F, LOW);  digitalWrite(in2F, LOW);
  digitalWrite(in3F, LOW);  digitalWrite(in4F, LOW);
  digitalWrite(in1B, LOW);  digitalWrite(in2B, LOW);
  digitalWrite(in3B, LOW);  digitalWrite(in4B, LOW);
}

/*
─────────────────────────────────────────────
CALIBRATION GUIDE
─────────────────────────────────────────────
1. Open Serial Monitor at 9600 baud
2. Hold the sensor over the colored line — note R, G, B values
3. Hold the sensor over the floor/background — note R, G, B values
4. Set COLOR_THRESHOLD to a value between the two readings
5. Set CLEAR_THRESHOLD low enough to pass when lit, high enough to fail in darkness

─────────────────────────────────────────────
UPGRADING TO PID (recommended for smooth following)
─────────────────────────────────────────────
Instead of stop/turn correction, compute an error value:
  error = (r - b)  // for red line: positive = on red, negative = off red
  correction = Kp * error;
  leftSpeed  = DRIVE_SPEED - correction;
  rightSpeed = DRIVE_SPEED + correction;
Clamp speeds to 0–255 and apply to each side independently.
Start with Kp = 0.5 and tune from there.
─────────────────────────────────────────────
*/
