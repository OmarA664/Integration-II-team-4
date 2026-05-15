
// Autonomous obstacle avoidance using Robot_Driving_Functions_v3 motor functions
// Teensy 4.1 pin variables — change wiring only here

// ===== SENSOR PINS =====
const int IR_LEFT_PIN = 22;
const int IR_RIGHT_PIN = 23;

const int TCS_S0 = 24;
const int TCS_S1 = 25;
const int TCS_S2 = 26;
const int TCS_S3 = 27;
const int TCS_OUT = 28;

const int FRONT_TRIG = 29;
const int FRONT_ECHO = 30;
const int RIGHT_TRIG = 31;
const int RIGHT_ECHO = 32;

// ===== MOTOR PINS (copied v3) =====
int enAF=10,in1F=2,in2F=3;
int enBF=11,in3F=4,in4F=5;
int enAB=12,in1B=6,in2B=7;
int enBB=13,in3B=8,in4B=9;

// ===== TUNING =====
int cruise=120;
int turnspd=100;
float wallStop=18;
float rightTarget=15;

// state machine for maze
int section=0;   //0 first wall, 1 middle gap, 2 last wall

void setup(){
 Serial.begin(115200);
 pinMode(IR_LEFT_PIN,INPUT);
 pinMode(IR_RIGHT_PIN,INPUT);
 pinMode(FRONT_TRIG,OUTPUT); pinMode(FRONT_ECHO,INPUT);
 pinMode(RIGHT_TRIG,OUTPUT); pinMode(RIGHT_ECHO,INPUT);

 pinMode(enAF,OUTPUT); pinMode(enBF,OUTPUT);
 pinMode(enAB,OUTPUT); pinMode(enBB,OUTPUT);
 pinMode(in1F,OUTPUT); pinMode(in2F,OUTPUT);
 pinMode(in3F,OUTPUT); pinMode(in4F,OUTPUT);
 pinMode(in1B,OUTPUT); pinMode(in2B,OUTPUT);
 pinMode(in3B,OUTPUT); pinMode(in4B,OUTPUT);
}

void loop(){
 float front=readUS(FRONT_TRIG,FRONT_ECHO);
 float right=readUS(RIGHT_TRIG,RIGHT_ECHO);

 // tape boundary avoidance
 if(boundarySeen()){ driveBackward(90); delay(200); turnRight(90); delay(250); return; }

 switch(section){
   case 0: // follow first wall on right
      wallFollow(right,front);
      if(front<wallStop){ turnRight(turnspd); delay(450); section=1; }
      break;

   case 1: // find opening between two walls
      driveForward(cruise);
      if(right>35){ section=2; } // opening detected
      break;

   case 2: // navigate last wall
      wallFollow(right,front);
      if(front<wallStop){ turnLeft(turnspd); delay(450); }
      break;
 }
}

// -------- sensors ----------
bool boundarySeen(){
 return digitalRead(IR_LEFT_PIN)==LOW || digitalRead(IR_RIGHT_PIN)==LOW;
}
float readUS(int trig,int echo){
 digitalWrite(trig,LOW); delayMicroseconds(2);
 digitalWrite(trig,HIGH); delayMicroseconds(10);
 digitalWrite(trig,LOW);
 return pulseIn(echo,HIGH)*0.034/2.0;
}

// -------- navigation --------
void wallFollow(float right,float front){
 if(front<wallStop){ turnLeft(turnspd); return; }

 // If the right wall is too close, steer left.
 // If the right wall is too far away, steer right.
 if(right<rightTarget-3){ turnLeft(80); }
 else if(right>rightTarget+3){ turnRight(80); }
 else driveForward(cruise);
}

// ===== original v3 drive functions =====
void driveForward(int s){analogWrite(enAF,s);analogWrite(enBF,s);analogWrite(enAB,s);analogWrite(enBB,s);
digitalWrite(in1F,LOW);digitalWrite(in2F,HIGH);digitalWrite(in3F,LOW);digitalWrite(in4F,HIGH);
digitalWrite(in1B,LOW);digitalWrite(in2B,HIGH);digitalWrite(in3B,LOW);digitalWrite(in4B,HIGH);}
void driveBackward(int s){analogWrite(enAF,s);analogWrite(enBF,s);analogWrite(enAB,s);analogWrite(enBB,s);
digitalWrite(in1F,HIGH);digitalWrite(in2F,LOW);digitalWrite(in3F,HIGH);digitalWrite(in4F,LOW);
digitalWrite(in1B,HIGH);digitalWrite(in2B,LOW);digitalWrite(in3B,HIGH);digitalWrite(in4B,LOW);}
void turnRight(int s){driveBackward(s); digitalWrite(in1B,LOW);digitalWrite(in2B,HIGH);}
void turnLeft(int s){driveForward(s); digitalWrite(in1B,HIGH);digitalWrite(in2B,LOW);}
