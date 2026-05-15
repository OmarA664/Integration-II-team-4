/*
  Robot_Wall_Avoidance_v3_RightSensor_DEBUG.ino

  What should happen:
  1) On reset/upload, robot waits 2 seconds.
  2) It drives forward briefly as a motor sanity test.
  3) It enters obstacle avoidance using:
     - RIGHT HC-SR04 for wall following
     - FRONT HC-SR04 for front obstacle detection
     - Left/right IR sensors for white tape boundary avoidance

  If the robot still does not move:
  - Open Serial Monitor at 115200 baud.
  - Check whether IR sensors are reading tape all the time.
  - Check whether ultrasonic readings are 999, meaning no echo was received.
*/

// ===================== SENSOR PIN VARIABLES =====================
// Change these if your Teensy 4.1 wiring changes.

const int IR_LEFT_PIN  = 22;   // HiLetGo IR sensor left of color sensor
const int IR_RIGHT_PIN = 23;   // HiLetGo IR sensor right of color sensor

const int TCS_S0  = 24;        // TCS3200 pins included for easy wiring updates
const int TCS_S1  = 25;
const int TCS_S2  = 26;
const int TCS_S3  = 27;
const int TCS_OUT = 28;

const int FRONT_TRIG = 29;     // Front HC-SR04 trigger
const int FRONT_ECHO = 30;     // Front HC-SR04 echo

const int RIGHT_TRIG = 31;     // Right HC-SR04 trigger
const int RIGHT_ECHO = 32;     // Right HC-SR04 echo

// ===================== MOTOR PINS FROM Robot_Driving_Functions_v3 =====================
// L298 Motor Controller Pin Assignment

// Motor A pin connections (Front Left)
int enAF = 10;
int in1F = 2;
int in2F = 3;

// Motor B pin connections (Front Right)
int enBF = 11;
int in3F = 4;
int in4F = 5;

// Motor C pin connections (Back Left)
int enAB = 12;
int in1B = 6;
int in2B = 7;

// Motor D pin connections (Back Right)
int enBB = 13;
int in3B = 8;
int in4B = 9;

// ===================== TUNING VARIABLES =====================

int cruiseSpeed = 150;          // Main forward speed, 0-255
int turnSpeed   = 110;          // Turning speed, 0-255
int backupSpeed = 110;

float frontWallStopDistance_cm = 18.0;  // Turn away if front wall is closer than this
float rightWallTarget_cm       = 15.0;  // Desired distance from right wall
float rightWallTolerance_cm    = 4.0;   // Allowed error before steering

float openingDistance_cm       = 35.0;  // If right sensor sees more than this, assume gap/opening
float noEchoDistance_cm        = 999.0; // Used when ultrasonic echo is missing

// For most small IR obstacle/line modules:
// LOW often means detected, HIGH often means not detected.
// If yours acts opposite, change this to HIGH.
const int IR_TAPE_DETECTED_STATE = LOW;

// Set this true after debugging if you do NOT want tape sensors to stop/steer the robot.
bool useTapeBoundaryAvoidance = true;

// ===================== STATE MACHINE =====================

int section = 0;
// section 0 = follow first wall using right sensor
// section 1 = drive toward / through middle opening
// section 2 = follow/navigate last wall

unsigned long lastPrintTime = 0;

// ===================== SETUP =====================

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);

  pinMode(TCS_S0, OUTPUT);
  pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);

  // TCS3200 frequency scaling. Not actively used in this first navigation version.
  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, LOW);

  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);

  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);

  pinMode(enAF, OUTPUT);  pinMode(enBF, OUTPUT);
  pinMode(in1F, OUTPUT);  pinMode(in2F, OUTPUT);
  pinMode(in3F, OUTPUT);  pinMode(in4F, OUTPUT);

  pinMode(enAB, OUTPUT);  pinMode(enBB, OUTPUT);
  pinMode(in1B, OUTPUT);  pinMode(in2B, OUTPUT);
  pinMode(in3B, OUTPUT);  pinMode(in4B, OUTPUT);

  stopMotors();

  Serial.println("Starting motor sanity test...");
  driveForward(cruiseSpeed);
  delay(700);
  stopMotors();
  delay(500);
  Serial.println("Entering obstacle avoidance.");
}

// ===================== MAIN LOOP =====================

void loop() {
  float frontDistance_cm = readUltrasonicCM(FRONT_TRIG, FRONT_ECHO);
  float rightDistance_cm = readUltrasonicCM(RIGHT_TRIG, RIGHT_ECHO);

  bool tapeSeen = boundarySeen();

  printDebug(frontDistance_cm, rightDistance_cm, tapeSeen);

  // White tape / boundary escape
  if (useTapeBoundaryAvoidance && tapeSeen) {
    stopMotors();
    delay(100);
    driveBackward(backupSpeed);
    delay(250);
    turnLeft(turnSpeed);      // using right-side wall following, left turn usually moves away from right boundary
    delay(300);
    stopMotors();
    delay(100);
    return;
  }

  switch (section) {
    case 0:
      // First wall: follow the right wall.
      wallFollowRight(rightDistance_cm, frontDistance_cm);

      // If a wall appears in front, turn left and move to middle section.
      if (frontDistance_cm < frontWallStopDistance_cm) {
        stopMotors();
        delay(100);
        turnLeft(turnSpeed);
        delay(450);
        stopMotors();
        section = 1;
      }
      break;

    case 1:
      // Middle barrier gap: move forward until right side looks open.
      driveForward(cruiseSpeed);

      if (rightDistance_cm > openingDistance_cm) {
        section = 2;
      }

      if (frontDistance_cm < frontWallStopDistance_cm) {
        stopMotors();
        delay(100);
        turnLeft(turnSpeed);
        delay(350);
      }
      break;

    case 2:
      // Final wall: continue with right wall following.
      wallFollowRight(rightDistance_cm, frontDistance_cm);

      if (frontDistance_cm < frontWallStopDistance_cm) {
        stopMotors();
        delay(100);
        turnLeft(turnSpeed);
        delay(450);
        stopMotors();
      }
      break;
  }
}

// ===================== SENSOR FUNCTIONS =====================

bool boundarySeen() {
  int leftIR  = digitalRead(IR_LEFT_PIN);
  int rightIR = digitalRead(IR_RIGHT_PIN);

  return (leftIR == IR_TAPE_DETECTED_STATE || rightIR == IR_TAPE_DETECTED_STATE);
}

float readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Timeout prevents the whole robot from freezing if no echo is received.
  // 30000 us is about 5 meters max range.
  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);

  if (duration == 0) {
    return noEchoDistance_cm;
  }

  return duration * 0.0343 / 2.0;
}

// ===================== NAVIGATION FUNCTIONS =====================

void wallFollowRight(float rightDistance_cm, float frontDistance_cm) {
  if (frontDistance_cm < frontWallStopDistance_cm) {
    turnLeft(turnSpeed);
    return;
  }

  // If no right wall is seen, keep moving forward.
  if (rightDistance_cm >= noEchoDistance_cm) {
    driveForward(cruiseSpeed);
    return;
  }

  // Right wall too close: steer left.
  if (rightDistance_cm < rightWallTarget_cm - rightWallTolerance_cm) {
    turnLeft(turnSpeed);
  }
  // Right wall too far: steer right.
  else if (rightDistance_cm > rightWallTarget_cm + rightWallTolerance_cm) {
    turnRight(turnSpeed);
  }
  // Right wall distance is good.
  else {
    driveForward(cruiseSpeed);
  }
}

void printDebug(float frontDistance_cm, float rightDistance_cm, bool tapeSeen) {
  if (millis() - lastPrintTime >= 250) {
    lastPrintTime = millis();

    Serial.print("section=");
    Serial.print(section);

    Serial.print("  front=");
    Serial.print(frontDistance_cm);

    Serial.print(" cm  right=");
    Serial.print(rightDistance_cm);

    Serial.print(" cm  IR_L=");
    Serial.print(digitalRead(IR_LEFT_PIN));

    Serial.print("  IR_R=");
    Serial.print(digitalRead(IR_RIGHT_PIN));

    Serial.print("  tapeSeen=");
    Serial.println(tapeSeen ? "YES" : "NO");
  }
}

// ===================== DRIVING FUNCTIONS COPIED FROM Robot_Driving_Functions_v3 =====================
// Speed: PWM value 0-255

void driveForward(int speed) {
  // Right side drives forward, left side drives forwards
  analogWrite(enAF, speed);
  analogWrite(enBF, speed);
  analogWrite(enAB, speed);
  analogWrite(enBB, speed);

  // Front Left - forward
  digitalWrite(in1F, LOW);
  digitalWrite(in2F, HIGH);

  // Front Right - forward
  digitalWrite(in3F, LOW);
  digitalWrite(in4F, HIGH);

  // Back Left - forward
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);

  // Back Right - forward
  digitalWrite(in3B, LOW);
  digitalWrite(in4B, HIGH);
}

void driveBackward(int speed) {
  // Left side drives backwards, right side drives backward
  analogWrite(enAF, speed);
  analogWrite(enBF, speed);
  analogWrite(enAB, speed);
  analogWrite(enBB, speed);

  // Front Left - backwards
  digitalWrite(in1F, HIGH);
  digitalWrite(in2F, LOW);

  // Front Right - backward
  digitalWrite(in3F, HIGH);
  digitalWrite(in4F, LOW);

  // Back Left - backwards
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);

  // Back Right - backward
  digitalWrite(in3B, HIGH);
  digitalWrite(in4B, LOW);
}

void turnRight(int speed) {
  // Left side drives forward, right side drives backward
  analogWrite(enAF, speed);
  analogWrite(enBF, speed);
  analogWrite(enAB, speed);
  analogWrite(enBB, speed);

  // Front Left - forward
  digitalWrite(in1F, HIGH);
  digitalWrite(in2F, LOW);

  // Front Right - backward
  digitalWrite(in3F, LOW);
  digitalWrite(in4F, HIGH);

  // Back Left - forward
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);

  // Back Right - backward
  digitalWrite(in3B, HIGH);
  digitalWrite(in4B, LOW);
}

void turnLeft(int speed) {
  // Left side drives backward, right side drives forward
  analogWrite(enAF, speed);
  analogWrite(enBF, speed);
  analogWrite(enAB, speed);
  analogWrite(enBB, speed);

  // Front Left - forward
  digitalWrite(in1F, LOW);
  digitalWrite(in2F, HIGH);

  // Front Right - backward
  digitalWrite(in3F, HIGH);
  digitalWrite(in4F, LOW);

  // Back Left - forward
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);

  // Back Right - backward
  digitalWrite(in3B, LOW);
  digitalWrite(in4B, HIGH);
}

void stopMotors() {
  digitalWrite(in1F, LOW);
  digitalWrite(in2F, LOW);
  digitalWrite(in3F, LOW);
  digitalWrite(in4F, LOW);

  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
  digitalWrite(in3B, LOW);
  digitalWrite(in4B, LOW);
}
