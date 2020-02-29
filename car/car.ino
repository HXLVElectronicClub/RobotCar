#define LEFT 8
#define RIGHT 9

void setup() {
  pinMode(LEFT, OUTPUT);
  pinMode(RIGHT, OUTPUT);
  digitalWrite(LEFT, 1);
  digitalWrite(RIGHT,0);
}

void loop() {
  
  digitalWrite(LEFT, 0);
  delay(1000);
  digitalWrite(LEFT,1);
  delay(1000);
}
