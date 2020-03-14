#define LEFT 8
#define RIGHT 9
#define SENSOR_LEFT A0
#define SENSOR_RIGHT A1
#define IRDATA 11

#include "IRremote.h"
#include "Move.h"

void IRControl();
void DoMove(long);

IRrecv irrecv(IRDATA);
decode_results results;

void setup() {
  Serial.begin(9600);
  pinMode(LEFT, OUTPUT);
  pinMode(RIGHT, OUTPUT);
  MotorPins(LEFT,RIGHT);
  irrecv.enableIRIn();
}

void loop() {
  //goAndStop();
  //trace();
  IRControl();
}

void goAndStop() {
  MoveForward(1000);
  Stop();
}

void trace() {
  int sensor1 = analogRead(SENSOR_LEFT);
  int sensor2 = analogRead(SENSOR_RIGHT);
  
  if (sensor1 > sensor2) {
    TurnLeft();
  } else {
    TurnRight();
  }
  delay(100);
}

long lastmove;
void IRControl() {
  if (irrecv.decode(&results)) {
    Serial.print("IR: ");
    Serial.println(results.value);
    if (results.value == 0xFFFFFFFF) {
        DoMove(lastmove );
     } else {
        DoMove(results.value); 
     }
     irrecv.resume(); 
  } else {
    Stop();
  }
  delay(150);
}

/*   
  case 0xFFA25D: Serial.println("POWER"); break;
  case 0xFFE21D: Serial.println("VOL STOP"); break;
  case 0xFF629D: Serial.println("MODE"); break;
  case 0xFF22DD: Serial.println("PAUSE");    break;
  case 0xFF02FD: Serial.println("FAST BACK");    break;
  case 0xFFC23D: Serial.println("FAST FORWARD");   break;
  case 0xFFE01F: Serial.println("EQ");    break;
  case 0xFFA857: Serial.println("VOL-");    break;
  case 0xFF906F: Serial.println("VOL+");    break;
  case 0xFF9867: Serial.println("RETURN");    break;
  case 0xFFB04F: Serial.println("USB SCAN");    break;
  case 0xFF6897: Serial.println("0");    break;
  case 0xFF30CF: Serial.println("1");    break;
  case 0xFF18E7: Serial.println("2");    break;
  case 0xFF7A85: Serial.println("3");    break;
  case 0xFF10EF: Serial.println("4");    break;
  case 0xFF38C7: Serial.println("5");    break;
  case 0xFF5AA5: Serial.println("6");    break;
  case 0xFF42BD: Serial.println("7");    break;
  case 0xFF4AB5: Serial.println("8");    break;
  case 0xFF52AD: Serial.println("9");    break;
  case 0xFFFFFFFF: Serial.println(" REPEAT");break;  
  */
void DoMove(long value) {
  switch(value) {
      case 0xFF18E7: // 2
        MoveForward();
        break;
      case 0xFF30CF: // 1
      case 0xFF10EF: // 4
        TurnLeft();
        break;
      case 0xFF38C7: // 5
        Stop();
        break;
      case 0xFF7A85: // 3
      case 0xFF5AA5: // 6
        TurnRight();
        break;
    }
  lastmove = value;
}
