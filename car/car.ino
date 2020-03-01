#define LEFT 8
#define RIGHT 9
#define SENSOR_LEFT A0
#define SENSOR_RIGHT A1

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
  int sensor1 = analogRead(SENSOR_LEFT);
  int sensor2 = analogRead(SENSOR_RIGHT);
  
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
