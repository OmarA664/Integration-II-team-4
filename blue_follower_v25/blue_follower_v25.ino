// ============================================================
//  BLUE LINE FOLLOWER
//  Arduino MEGA | L298N x2 | TCS3200 | 2x IR | 2x Button
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

// --- Buttons
const int PIN_BTN_SMALL = 33;  // Small box = left fork
const int PIN_BTN_LARGE = 34;  // Large box = right fork

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
//  SPEED SETTINGS
// ============================================================
const int SPEED_FORWARD = 130;
const int SPEED_TURN    = 130;

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
//  STATE MACHINE
// ============================================================
enum RobotState {
  STATE_WAIT_BUTTON,    // 0 - waiting for button press
  STATE_FIND_GREEN,     // 1 - drive forward until green + IR 1 1
  STATE_ALIGN_GREEN,    // 2 - turn right until IR 0 0 (aligned on green)
  STATE_FOLLOW_GREEN,   // 3 - drive along green until left IR = 1
  STATE_TURN_TO_BLUE,   // 4 - turn left until color sensor sees blue
  STATE_FOLLOW_BLUE,    // 5 - follow blue line
  STATE_FORK_SMALL,     // 6 - small box: turn left, follow blue 2s, stop
  STATE_FORK_LARGE,     // 7 - large box: turn right, follow blue 2s, stop
  STATE_FIND_FORK,      // 8 - drive forward after green until blue + IR 0 0
  STATE_DONE            // 9 - stopped
};

RobotState robotState = STATE_WAIT_BUTTON;
bool smallBox = false;
bool followRed = false;
unsigned long forkTimer = 0;
unsigned long greenFollowStart = 0;
unsigned long blueFollowStart = 0;
const unsigned long GREEN_FOLLOW_MIN_MS = 4000;
const unsigned long BLUE_FOLLOW_MIN_MS  = 3000;  // must follow blue for at least 3s before fork detection

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
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
  pinMode(PIN_BTN_SMALL, INPUT_PULLUP);
  pinMode(PIN_BTN_LARGE, INPUT_PULLUP);

  // Use multiple analog pins and millis for better entropy
  stopMotors();
  Serial.println("Choose line color: Press SMALL button (pin 33) for RED, LARGE button (pin 34) for BLUE.");
  while (true) {
    if (digitalRead(PIN_BTN_SMALL) == LOW) {
      followRed = true;
      Serial.println("RED selected.");
      delay(300);
      break;
    } else if (digitalRead(PIN_BTN_LARGE) == LOW) {
      followRed = false;
      Serial.println("BLUE selected.");
      delay(300);
      break;
    }
  }
  Serial.println("Now choose box size: Press SMALL (pin 33) for small, LARGE (pin 34) for large.");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  int r, g, b;
  readColorSensor(r, g, b);
  LineColor color = detectColor(r, g, b);

  int irLeft  = digitalRead(PIN_IR_LEFT);
  int irRight = digitalRead(PIN_IR_RIGHT);

  // State name lookup
  String stateName;
  switch(robotState) {
    case 0: stateName = "WAIT_BUTTON";  break;
    case 1: stateName = "FIND_GREEN";   break;
    case 2: stateName = "ALIGN_GREEN";  break;
    case 3: stateName = "FOLLOW_GREEN"; break;
    case 4: stateName = "TURN_TO_BLUE"; break;
    case 5: stateName = "FOLLOW_BLUE";  break;
    case 6: stateName = "FORK_SMALL";   break;
    case 7: stateName = "FORK_LARGE";   break;
    case 8: stateName = "FIND_FORK";    break;
    case 9: stateName = "DONE";         break;
    default: stateName = "UNKNOWN";
  }
  String colorName;
  switch(color) {
    case COLOR_BLUE:    colorName = "BLUE";    break;
    case COLOR_RED:     colorName = "RED";     break;
    case COLOR_GREEN:   colorName = "GREEN";   break;
    case COLOR_UNKNOWN: colorName = "UNKNOWN"; break;
    default:            colorName = "OTHER";
  }
  Serial.print("["); Serial.print(stateName); Serial.print("] ");
  Serial.print("Color="); Serial.print(colorName);
  Serial.print(" R="); Serial.print(r);
  Serial.print(" G="); Serial.print(g);
  Serial.print(" B="); Serial.print(b);
  Serial.print(" | IR L="); Serial.print(irLeft);
  Serial.print(" R="); Serial.println(irRight);

  switch (robotState) {

    // ----------------------------------------------------------
    // 0: Wait for button
    // ----------------------------------------------------------
    case STATE_WAIT_BUTTON:
      stopMotors();
      if (digitalRead(PIN_BTN_SMALL) == LOW) {
        smallBox = true;
        Serial.println("Small box selected.");
        delay(300);
        robotState = STATE_FIND_GREEN;
      } else if (digitalRead(PIN_BTN_LARGE) == LOW) {
        smallBox = false;
        Serial.println("Large box selected.");
        delay(300);
        robotState = STATE_FIND_GREEN;
      }
      break;

    // ----------------------------------------------------------
    // 1: Drive forward until green + IR 0 0 (perpendicular crossing)
    // ----------------------------------------------------------
    case STATE_FIND_GREEN:
      if (color == COLOR_GREEN && irLeft == 0 && irRight == 0) {
        Serial.println("Green found perpendicularly (IR 0 0) — turning right to align.");
        stopMotors();
        delay(100);
        robotState = STATE_ALIGN_GREEN;
      } else {
        moveForward();
      }
      break;

    // ----------------------------------------------------------
    // 2: Turn right until IR 1 1 (now aligned along green)
    // ----------------------------------------------------------
    case STATE_ALIGN_GREEN:
      if (irLeft == 1 && irRight == 1) {
        Serial.println("IR 1 1 — aligned on green, driving along it.");
        stopMotors();
        delay(100);
        greenFollowStart = millis();
        robotState = STATE_FOLLOW_GREEN;
      } else {
        if (followRed) turnLeft();   // red is to the left
        else           turnRight();  // blue is to the right
      }
      break;

    // ----------------------------------------------------------
    // 3: Follow green line using IR steering (same logic as red follower)
    //    After min time, if left IR = 1 → reached blue junction
    // ----------------------------------------------------------
    case STATE_FOLLOW_GREEN: {
      bool minTimeElapsed = (millis() - greenFollowStart >= GREEN_FOLLOW_MIN_MS);

      // Check for junction only after min time
      // Blue: left IR = 1 means blue line is to the left
      // Red:  right IR = 1 means red line is to the right
      if (followRed) {
        if (minTimeElapsed && irRight == 1) {
          Serial.println("Right IR = 1 after min time — turning right to find red.");
          stopMotors();
          delay(100);
          robotState = STATE_TURN_TO_BLUE;
          break;
        }
      } else {
        if (minTimeElapsed && irLeft == 1) {
          Serial.println("Left IR = 1 after min time — turning left to find blue.");
          stopMotors();
          delay(100);
          robotState = STATE_TURN_TO_BLUE;
          break;
        }
      }

      // Follow green using inverted IR steering
      if (color == COLOR_GREEN) {
        if (irLeft == 1 && irRight == 1) {
          Serial.println("GREEN — forward");
          moveForward();
        } else if (irLeft == 0 && irRight == 1) {
          Serial.println("GREEN — correct left");
          turnLeft();
        } else if (irLeft == 1 && irRight == 0) {
          Serial.println("GREEN — correct right");
          turnRight();
        } else {
          Serial.println("GREEN — both on tape, forward");
          moveForward();
        }
      } else {
        // Off green — use IR to steer back
        if (irLeft == 0 && irRight == 1) {
          Serial.println("OFF GREEN — correct left");
          turnLeft();
        } else if (irLeft == 1 && irRight == 0) {
          Serial.println("OFF GREEN — correct right");
          turnRight();
        } else {
          Serial.println("OFF GREEN — forward");
          moveForward();
        }
      }
      break;
    }

    // ----------------------------------------------------------
    // 4: Pulse toward target color — left for blue, right for red
    // ----------------------------------------------------------
    case STATE_TURN_TO_BLUE:
      if (followRed) {
        if (color == COLOR_RED) {
          Serial.println("Red found — following red line.");
          stopMotors();
          delay(100);
          blueFollowStart = millis();
          robotState = STATE_FOLLOW_BLUE;
        } else {
          Serial.println("Pulsing right for red...");
          turnRight();
          delay(50);
          stopMotors();
          delay(30);
        }
      } else {
        if (color == COLOR_BLUE) {
          Serial.println("Blue found — following blue line.");
          stopMotors();
          delay(100);
          blueFollowStart = millis();
          robotState = STATE_FOLLOW_BLUE;
        } else {
          Serial.println("Pulsing left for blue...");
          turnLeft();
          delay(50);
          stopMotors();
          delay(30);
        }
      }
      break;

    // ----------------------------------------------------------
    // 5: Follow blue line — fork detected when blue + IR 1 1
    // ----------------------------------------------------------
    case STATE_FOLLOW_BLUE: {
      LineColor target = followRed ? COLOR_RED : COLOR_BLUE;
      if (color == COLOR_GREEN && irLeft == 0 && irRight == 0) {
        Serial.println("Green crossing detected — driving forward to fork.");
        stopMotors();
        delay(100);
        robotState = STATE_FIND_FORK;
        break;
      }
      if (color == target) {
        if (irLeft == 1 && irRight == 1) {
          moveForward();
        } else if (irLeft == 0 && irRight == 1) {
          turnLeft();
        } else if (irLeft == 1 && irRight == 0) {
          turnRight();
        } else {
          moveForward();
        }
      } else {
        if (irLeft == 0 && irRight == 1) {
          turnLeft();
        } else if (irLeft == 1 && irRight == 0) {
          turnRight();
        } else {
          moveForward();
        }
      }
      break;
    }


    // ----------------------------------------------------------
    // 6: Small box — turn left on blue 0 0, follow until unknown + IR 1 1
    // ----------------------------------------------------------
    case STATE_FORK_SMALL:
      // Stop condition: unknown color + both IR black
      if (color == COLOR_UNKNOWN && irLeft == 1 && irRight == 1 &&
          millis() - forkTimer >= 2000) {
        Serial.println("End of fork — stopping.");
        stopMotors();
        robotState = STATE_DONE;
        break;
      }
      // Turn left when both on target color (at junction), otherwise follow
      { LineColor ft = followRed ? COLOR_RED : COLOR_BLUE;
      if (color == ft && irLeft == 0 && irRight == 0) {
        Serial.println("FORK_SMALL — target 0 0, turning left.");
        turnLeft();
      } else if (color == ft) {
        if (irLeft == 1 && irRight == 1) moveForward();
        else if (irLeft == 0 && irRight == 1) turnLeft();
        else if (irLeft == 1 && irRight == 0) turnRight();
        else moveForward();
      } else {
        moveForward();
      }}
      break;

    // ----------------------------------------------------------
    // 7: Large box — turn right on blue 0 0, follow until unknown + IR 1 1
    // ----------------------------------------------------------
    case STATE_FORK_LARGE:
      // Stop condition: unknown color + both IR black
      if (color == COLOR_UNKNOWN && irLeft == 1 && irRight == 1 &&
          millis() - forkTimer >= 2000) {
        Serial.println("End of fork — stopping.");
        stopMotors();
        robotState = STATE_DONE;
        break;
      }
      // Turn right when both on target color (at junction), otherwise follow
      { LineColor ft = followRed ? COLOR_RED : COLOR_BLUE;
      if (color == ft && irLeft == 0 && irRight == 0) {
        Serial.println("FORK_LARGE — target 0 0, turning right.");
        turnRight();
      } else if (color == ft) {
        if (irLeft == 1 && irRight == 1) moveForward();
        else if (irLeft == 0 && irRight == 1) turnLeft();
        else if (irLeft == 1 && irRight == 0) turnRight();
        else moveForward();
      } else {
        moveForward();
      }}
      break;

    // ----------------------------------------------------------
    // 8: Follow blue line after green crossing until blue + IR 0 0
    //    (both sensors on blue = at the fork junction)
    // ----------------------------------------------------------
    case STATE_FIND_FORK: {
      LineColor forkTarget = followRed ? COLOR_RED : COLOR_BLUE;
      if (color == forkTarget && irLeft == 0 && irRight == 0) {
        Serial.println("Fork junction reached — executing choice.");
        stopMotors();
        delay(100);
        forkTimer = millis();
        robotState = smallBox ? STATE_FORK_SMALL : STATE_FORK_LARGE;
        break;
      }
      // Follow target color normally
      if (color == forkTarget) {
        if (irLeft == 1 && irRight == 1) {
          Serial.println("FIND_FORK — forward");
          moveForward();
        } else if (irLeft == 0 && irRight == 1) {
          Serial.println("FIND_FORK — correct left");
          turnLeft();
        } else if (irLeft == 1 && irRight == 0) {
          Serial.println("FIND_FORK — correct right");
          turnRight();
        } else {
          Serial.println("FIND_FORK — both on tape, forward");
          moveForward();
        }
      } else {
        Serial.println("FIND_FORK — off color, forward");
        moveForward();
      }
      break;
    }

    // ----------------------------------------------------------
    // 9: Done
    // ----------------------------------------------------------
    case STATE_DONE:
      stopMotors();
      break;
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
//  MOTOR FUNCTIONS
// ============================================================
void moveForward() {
  analogWrite(PIN_FL_EN, SPEED_FORWARD);
  analogWrite(PIN_FR_EN, SPEED_FORWARD);
  analogWrite(PIN_RL_EN, SPEED_FORWARD);
  analogWrite(PIN_RR_EN, SPEED_FORWARD);
  digitalWrite(PIN_FL_IN1, LOW);  digitalWrite(PIN_FL_IN2, HIGH);
  digitalWrite(PIN_FR_IN1, LOW);  digitalWrite(PIN_FR_IN2, HIGH);
  digitalWrite(PIN_RL_IN1, LOW);  digitalWrite(PIN_RL_IN2, HIGH);
  digitalWrite(PIN_RR_IN1, LOW);  digitalWrite(PIN_RR_IN2, HIGH);
}

void turnLeft() {
  analogWrite(PIN_FL_EN, SPEED_TURN);
  analogWrite(PIN_FR_EN, SPEED_TURN);
  analogWrite(PIN_RL_EN, SPEED_TURN);
  analogWrite(PIN_RR_EN, SPEED_TURN);
  digitalWrite(PIN_FL_IN1, LOW);  digitalWrite(PIN_FL_IN2, HIGH);
  digitalWrite(PIN_FR_IN1, HIGH); digitalWrite(PIN_FR_IN2, LOW);
  digitalWrite(PIN_RL_IN1, HIGH); digitalWrite(PIN_RL_IN2, LOW);
  digitalWrite(PIN_RR_IN1, LOW);  digitalWrite(PIN_RR_IN2, HIGH);
}

void turnRight() {
  analogWrite(PIN_FL_EN, SPEED_TURN);
  analogWrite(PIN_FR_EN, SPEED_TURN);
  analogWrite(PIN_RL_EN, SPEED_TURN);
  analogWrite(PIN_RR_EN, SPEED_TURN);
  digitalWrite(PIN_FL_IN1, HIGH); digitalWrite(PIN_FL_IN2, LOW);
  digitalWrite(PIN_FR_IN1, LOW);  digitalWrite(PIN_FR_IN2, HIGH);
  digitalWrite(PIN_RL_IN1, LOW);  digitalWrite(PIN_RL_IN2, HIGH);
  digitalWrite(PIN_RR_IN1, HIGH); digitalWrite(PIN_RR_IN2, LOW);
}

void stopMotors() {
  analogWrite(PIN_FL_EN, 0); analogWrite(PIN_FR_EN, 0);
  analogWrite(PIN_RL_EN, 0); analogWrite(PIN_RR_EN, 0);
  digitalWrite(PIN_FL_IN1, LOW); digitalWrite(PIN_FL_IN2, LOW);
  digitalWrite(PIN_FR_IN1, LOW); digitalWrite(PIN_FR_IN2, LOW);
  digitalWrite(PIN_RL_IN1, LOW); digitalWrite(PIN_RL_IN2, LOW);
  digitalWrite(PIN_RR_IN1, LOW); digitalWrite(PIN_RR_IN2, LOW);
}
