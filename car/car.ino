#define LEFT 8
#define RIGHT 9

void setup() {
  Serial.begin(9600);
  pinMode(LEFT, OUTPUT);
  pinMode(RIGHT, OUTPUT);
  digitalWrite(LEFT, 0);
  digitalWrite(RIGHT,0);
}

void loop() {
  goAndStop();
  //trace();
}

void goAndStop() {
  digitalWrite(LEFT, 0);
  digitalWrite(RIGHT, 0);
  delay(1000);
  digitalWrite(LEFT, 1);
  digitalWrite(RIGHT, 1);
  delay(1000);
}

void trace() {
  int sensor1 = analogRead(A0);
  int sensor2 = analogRead(A1);
  
  Serial.print("Sensor1:");
  Serial.print(sensor1);
  Serial.print("Sensor2:");
  Serial.print(sensor2);
  Serial.println();
  
  if (sensor1 > sensor2) {
    digitalWrite(LEFT, 0);
    digitalWrite(RIGHT, 1);
  } else {
    digitalWrite(LEFT, 1);
    digitalWrite(RIGHT, 0);    
  }
  delay(100);
}
