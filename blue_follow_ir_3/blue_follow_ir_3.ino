// ============================================================
//  BLUE LINE FOLLOWER — Color Sensor Only
//  Arduino MEGA | L298N x2 | TCS3200
// ============================================================

// --- TCS3200 Color Sensor
#define S0        14
#define S1        15
#define S2        16
#define S3        17
#define sensorOut 18

// --- IR Sensors
const int PIN_IR_LEFT  = 25;
const int PIN_IR_RIGHT = 24;

// --- Motor Driver — Direction pins
const int PIN_FL_IN1 = 2;
const int PIN_FL_IN2 = 3;
const int PIN_FR_IN1 = 4;
const int PIN_FR_IN2 = 5;
const int PIN_RL_IN1 = 6;
const int PIN_RL_IN2 = 7;
const int PIN_RR_IN1 = 8;
const int PIN_RR_IN2 = 9;

// --- Motor Driver — PWM Enable pins
const int PIN_FL_EN = 10;
const int PIN_FR_EN = 11;
const int PIN_RL_EN = 12;
const int PIN_RR_EN = 13;

// ============================================================
//  SPEED SETTINGS  (0–255)
// ============================================================
const int SPEED_FORWARD = 130;
const int SPEED_TURN    = 130;
const int SPEED_SLOW    = 110;

// ============================================================
//  COLOR THRESHOLDS
//  Measured pulse-width readings (lower = more of that color):
//    On BLUE:  R=299  G=195  B=120
//    On RED:   R=114  G=326  B=267
//    On GREEN: R=139  G=120  B=160
// ============================================================

const int BLUE_R_MIN = 220;  const int BLUE_R_MAX = 380;
const int BLUE_G_MIN = 130;  const int BLUE_G_MAX = 260;
const int BLUE_B_MIN =  70;  const int BLUE_B_MAX = 170;

const int RED_R_MIN =  60;   const int RED_R_MAX = 170;
const int RED_G_MIN = 230;   const int RED_G_MAX = 420;
const int RED_B_MIN = 180;   const int RED_B_MAX = 350;

const int GREEN_R_MIN =  85;  const int GREEN_R_MAX = 195;
const int GREEN_G_MIN =  70;  const int GREEN_G_MAX = 170;
const int GREEN_B_MIN = 100;  const int GREEN_B_MAX = 220;

// ============================================================
//  COLOR ENUM
// ============================================================
enum LineColor { COLOR_UNKNOWN, COLOR_BLUE, COLOR_RED, COLOR_GREEN };

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // 20% frequency scaling
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  pinMode(PIN_FL_IN1, OUTPUT); pinMode(PIN_FL_IN2, OUTPUT);
  pinMode(PIN_FR_IN1, OUTPUT); pinMode(PIN_FR_IN2, OUTPUT);
  pinMode(PIN_RL_IN1, OUTPUT); pinMode(PIN_RL_IN2, OUTPUT);
  pinMode(PIN_RR_IN1, OUTPUT); pinMode(PIN_RR_IN2, OUTPUT);
  pinMode(PIN_FL_EN,  OUTPUT); pinMode(PIN_FR_EN,  OUTPUT);
  pinMode(PIN_RL_EN,  OUTPUT); pinMode(PIN_RR_EN,  OUTPUT);

  pinMode(PIN_IR_LEFT,  INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);

  stopMotors();
  Serial.println("Ready — following blue.");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  // --- Read color sensor
  int r, g, b;
  readColorSensor(r, g, b);
  LineColor color = detectColor(r, g, b);

  // --- Read IR sensors
  int irLeft  = digitalRead(PIN_IR_LEFT);
  int irRight = digitalRead(PIN_IR_RIGHT);

  Serial.print("R="); Serial.print(r);
  Serial.print(" G="); Serial.print(g);
  Serial.print(" B="); Serial.print(b);
  Serial.print(" | IR L="); Serial.print(irLeft);
  Serial.print(" R="); Serial.print(irRight);
  Serial.print(" -> ");

  // --- GREEN: always drive through
  if (color == COLOR_GREEN) {
    Serial.println("GREEN — forward");
    moveForward();
    return;
  }

  // IR convention: 1 = black floor (correct path), 0 = seeing color/bright (off path)
  // IR sensors should straddle the EDGES of the blue tape.
  // When centered: both read 1 (black on either side of tape).
  // When drifting: the sensor crossing onto tape reads 0.

  // --- BLUE: use IR sensors to fine-tune steering
  if (color == COLOR_BLUE) {
    if (irLeft == 1 && irRight == 1) {
      // Both on black edges — centered on tape
      Serial.println("BLUE — forward");
      moveForward();
    } else if (irLeft == 0 && irRight == 1) {
      // Left crossed onto tape — drifted left, correct right
      Serial.println("BLUE — correct right");
      turnRight();
    } else if (irLeft == 1 && irRight == 0) {
      // Right crossed onto tape — drifted right, correct left
      Serial.println("BLUE — correct left");
      turnLeft();
    } else {
      // Both on tape — centered over tape, go forward
      Serial.println("BLUE — both on tape, forward");
      moveForward();
    }
    return;
  }

  // --- RED: fully off blue, use IR to steer back
  if (color == COLOR_RED) {
    if (irLeft == 0 && irRight == 1) {
      Serial.println("RED — correct right");
      turnRight();
    } else {
      Serial.println("RED — correct left");
      turnLeft();
    }
    return;
  }

  // --- UNKNOWN: trust IR sensors alone
  if (irLeft == 1 && irRight == 1) {
    Serial.println("UNKNOWN — both on black, forward");
    moveForward();
  } else if (irLeft == 0 && irRight == 1) {
    Serial.println("UNKNOWN — correct right");
    turnRight();
  } else if (irLeft == 1 && irRight == 0) {
    Serial.println("UNKNOWN — correct left");
    turnLeft();
  } else {
    Serial.println("UNKNOWN — line lost, stop");
    stopMotors();
  }
}

// ============================================================
//  COLOR SENSOR
// ============================================================
int readChannel(int s2Val, int s3Val) {
  digitalWrite(S2, s2Val);
  digitalWrite(S3, s3Val);
  delay(10);
  return (int) pulseIn(sensorOut, LOW, 100000);
}

void readColorSensor(int &r, int &g, int &b) {
  r = readChannel(LOW,  LOW);
  g = readChannel(HIGH, HIGH);
  b = readChannel(LOW,  HIGH);
}

LineColor detectColor(int r, int g, int b) {
  if (r >= BLUE_R_MIN  && r <= BLUE_R_MAX  &&
      g >= BLUE_G_MIN  && g <= BLUE_G_MAX  &&
      b >= BLUE_B_MIN  && b <= BLUE_B_MAX)  return COLOR_BLUE;

  if (r >= RED_R_MIN   && r <= RED_R_MAX   &&
      g >= RED_G_MIN   && g <= RED_G_MAX   &&
      b >= RED_B_MIN   && b <= RED_B_MAX)   return COLOR_RED;

  if (r >= GREEN_R_MIN && r <= GREEN_R_MAX &&
      g >= GREEN_G_MIN && g <= GREEN_G_MAX &&
      b >= GREEN_B_MIN && b <= GREEN_B_MAX) return COLOR_GREEN;

  return COLOR_UNKNOWN;
}

// ============================================================
//  MOTOR FUNCTIONS  (logic from Robot_Driving_Functions_v3)
//  turnLeft/turnRight are swapped intentionally to match
//  physical robot orientation.
// ============================================================

void moveForward() {
  analogWrite(PIN_FL_EN, SPEED_FORWARD);
  analogWrite(PIN_FR_EN, SPEED_FORWARD);
  analogWrite(PIN_RL_EN, SPEED_FORWARD);
  analogWrite(PIN_RR_EN, SPEED_FORWARD);

  digitalWrite(PIN_FL_IN1, LOW);  digitalWrite(PIN_FL_IN2, HIGH); // FL forward
  digitalWrite(PIN_FR_IN1, LOW);  digitalWrite(PIN_FR_IN2, HIGH); // FR forward
  digitalWrite(PIN_RL_IN1, LOW);  digitalWrite(PIN_RL_IN2, HIGH); // RL forward
  digitalWrite(PIN_RR_IN1, LOW);  digitalWrite(PIN_RR_IN2, HIGH); // RR forward
}

// Swapped: this physically turns the robot RIGHT
void turnLeft() {
  analogWrite(PIN_FL_EN, SPEED_TURN);
  analogWrite(PIN_FR_EN, SPEED_TURN);
  analogWrite(PIN_RL_EN, SPEED_TURN);
  analogWrite(PIN_RR_EN, SPEED_TURN);

  digitalWrite(PIN_FL_IN1, HIGH); digitalWrite(PIN_FL_IN2, LOW);  // FL backward
  digitalWrite(PIN_FR_IN1, LOW);  digitalWrite(PIN_FR_IN2, HIGH); // FR forward
  digitalWrite(PIN_RL_IN1, LOW);  digitalWrite(PIN_RL_IN2, HIGH); // RL backward
  digitalWrite(PIN_RR_IN1, HIGH); digitalWrite(PIN_RR_IN2, LOW);  // RR forward
}

// Swapped: this physically turns the robot LEFT
void turnRight() {
  analogWrite(PIN_FL_EN, SPEED_TURN);
  analogWrite(PIN_FR_EN, SPEED_TURN);
  analogWrite(PIN_RL_EN, SPEED_TURN);
  analogWrite(PIN_RR_EN, SPEED_TURN);

  digitalWrite(PIN_FL_IN1, LOW);  digitalWrite(PIN_FL_IN2, HIGH); // FL forward
  digitalWrite(PIN_FR_IN1, HIGH); digitalWrite(PIN_FR_IN2, LOW);  // FR backward
  digitalWrite(PIN_RL_IN1, HIGH); digitalWrite(PIN_RL_IN2, LOW);  // RL forward
  digitalWrite(PIN_RR_IN1, LOW);  digitalWrite(PIN_RR_IN2, HIGH); // RR backward
}

void stopMotors() {
  analogWrite(PIN_FL_EN, 0); analogWrite(PIN_FR_EN, 0);
  analogWrite(PIN_RL_EN, 0); analogWrite(PIN_RR_EN, 0);
  digitalWrite(PIN_FL_IN1, LOW); digitalWrite(PIN_FL_IN2, LOW);
  digitalWrite(PIN_FR_IN1, LOW); digitalWrite(PIN_FR_IN2, LOW);
  digitalWrite(PIN_RL_IN1, LOW); digitalWrite(PIN_RL_IN2, LOW);
  digitalWrite(PIN_RR_IN1, LOW); digitalWrite(PIN_RR_IN2, LOW);
}
