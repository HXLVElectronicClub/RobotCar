// comment this line out if you are not using L239D and drive the motor directly.
#define L239D_DRIVE

/*-------------------------------
   Define used pins
-------------------------------*/
#ifndef L239D_DRIVE
  #define LEFT 8
  #define RIGHT 9
#else
  #define LEFT 5
  #define LEFTR 4
  #define RIGHT 3
  #define RIGHTR 2
  #define ENABLE_LEFT 9
  #define ENABLE_RIGHT 6
#endif

#define SENSOR_LEFT A0
#define SENSOR_RIGHT A1
#define IRDATA 11
#define ECHO_PIN 8
#define TRIG_PIN 7
/*-------------------------------*/

/*---------------------------------
 * include libraries
 *---------------------------------*/
#include "IRremote.h"
#include "Move.h"
#include "SR04.h"

/*---------------------------------
 * Function declaration
 *--------------------------------*/
// Read IR sensor, and call coresponding function
void IRControl();
// Move according to IR data
void DoMove(long);
// Auto move, avoid obstacal by distance sensor
void AutoMove();

/*---------------------------------
 * Controller/sensor instance
 *--------------------------------*/
IRrecv irrecv(IRDATA);
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
/*----------------------------------
 * code start
 *--------------------------------*/
void setup() {
  Serial.begin(9600);
  pinMode(LEFT, OUTPUT);
  pinMode(RIGHT, OUTPUT);
#ifndef L239D_DRIVE
  MotorPins(LEFT,RIGHT);
#else
  pinMode(LEFTR, OUTPUT);
  pinMode(RIGHTR, OUTPUT);
  MotorPins(LEFT, LEFTR, RIGHT, RIGHTR, ENABLE_LEFT, ENABLE_RIGHT); 
#endif
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
bool automove=false;
void IRControl() {
  decode_results results;
  if (irrecv.decode(&results)) {
    Serial.print("IR: 0x");
    Serial.println(results.value, HEX);
    if (results.value == 0xFF629D) { // "MODE"
      automove = !automove;
    } else if (results.value == 0xFFFFFFFF) {
        DoMove(lastmove );
     } else {
        DoMove(results.value);
        lastmove = results.value; 
     }
     irrecv.resume(); 
  } else if (automove) {
    AutoMove();
  } else {
    Stop();
  }
  delay(100);
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
        Serial.println("MoveFW:");
        MoveForward();
        break;
      case 0xFF30CF: // 1
      case 0xFF10EF: // 4
        TurnLeft();
        Serial.println("MoveLT:");
        break;
      case 0xFF38C7: // 5
        MoveBackward();
        Serial.println("MOVEBW:");
        break;
      case 0xFF7A85: // 3
      case 0xFF5AA5: // 6
        Serial.println("MoveRT:");
        TurnRight();
        break;
    }
}


void AutoMove() {
  long a = sr04.Distance();
  Serial.print(a);
  Serial.println("cm");
  if (a <= 15) {
    Stop();
    MoveBackward(100);
    TurnLeft();
  } else {
    MoveForward();
  }
}
