int irOutputLeft = 25;
int irOutputRight = 24;

int detectionStateLeft = 1;
int detectionStateRight = 1;

void setup() {
  Serial.begin(9600);

  pinMode(irOutputLeft, INPUT);
  pinMode(irOutputRight, INPUT);


}

void loop() {
  detectionStateLeft = digitalRead(irOutputLeft);
  detectionStateRight = digitalRead(irOutputRight);
  
  Serial.print("Left Sensor = ");
  Serial.print(detectionStateLeft);
  Serial.print(" ––––––––– Right Sensor = ");
  Serial.println(detectionStateRight);
  delay(500);

}
