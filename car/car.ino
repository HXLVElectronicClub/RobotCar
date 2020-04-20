// comment this line out if you are not using L239D and drive the motor directly.
#define L239D_DRIVE
// use shift register to save pins
#define USE_74HC595

// Select IR or Bluetooth
//#define USE_IR
#define USE_BLUETOOTH
// push demo
/*-------------------------------
   Define used pins
-------------------------------*/
#ifndef L239D_DRIVE
  #define LEFT 8
  #define RIGHT 9
#else
    #define LEFT         5
    #define LEFTR        4
    #define RIGHT        3
    #define RIGHTR       2
    #define ENABLE_LEFT  9
    #define ENABLE_RIGHT 6
#endif

#ifdef USE_74HC595
    #define SHIFT_IN    2
    #define SHIFT_CLK   3
    #define SHIFT_LATCH 4
#endif

#define SENSOR_LEFT A0
#define SENSOR_RIGHT A1
#define IRDATA 11
#define ECHO_PIN 8
#define TRIG_PIN 7

#ifdef USE_BLUETOOTH
  #define BT_TX 12
  #define BT_RX 13
#endif
/*-------------------------------*/

/*---------------------------------
 * include libraries
 *---------------------------------*/
#ifdef USE_BLUETOOTH
  #include "ArduinoBlue.h"
  #include <SoftwareSerial.h>
#endif
#ifdef USE_IR
  #include "IRremote.h"
#endif

#include "Move.h"
//Distance sensor
#include "SR04.h"

/*---------------------------------
 * Function declaration
 *--------------------------------*/
// Read IR sensor/or bluetooth control, and call coresponding function
#ifdef USE_IR
void IRControl();
#endif
#ifdef USE_BLUETOOTH
void BTControl();
#endif
// Move according to IR data
bool DoMove(long);
// Auto move, avoid obstacal by distance sensor
void AutoMove();

/*---------------------------------
 * Controller/sensor instance
 *--------------------------------*/
#ifdef USE_IR
  IRrecv irrecv(IRDATA);
#endif
#ifdef USE_BLUETOOTH
  SoftwareSerial softSerial(BT_TX,BT_RX);
  ArduinoBlue phone(softSerial);
#endif
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
/*----------------------------------
 * code start
 *--------------------------------*/
void setup() {
  Serial.begin(9600);
#ifndef L239D_DRIVE
  MotorPins(LEFT,RIGHT);
#else
  #ifdef USE_74HC595
    MotorPins_shift(SHIFT_IN,SHIFT_CLK,SHIFT_LATCH,ENABLE_LEFT,ENABLE_RIGHT);
  #else
    MotorPins(LEFT, LEFTR, RIGHT, RIGHTR, ENABLE_LEFT, ENABLE_RIGHT);
  #endif 
  SetSpeedRatio(1,0.9);
#endif
#ifdef USE_IR
  irrecv.enableIRIn();
#endif
#ifdef USE_BLUETOOTH
  softSerial.begin(9600);
#endif
}

void loop() {
#ifdef USE_BLUETOOTH
  BTControl();
#endif
#ifdef USE_IR
  IRControl();
#endif
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

bool automove=false;

#ifdef USE_IR
bool nocmd=true;
long t = 0;
void IRControl() {
  decode_results results;
  if (irrecv.decode(&results)) {
    Serial.print("IR: 0x");
    Serial.println(results.value, HEX);
    
    if (DoMove(results.value)) {
      t=millis();
      nocmd = false;
    }
    irrecv.resume();
  } else if (automove) {
    Serial.println("AutoMove");
    AutoMove();
  } else if (!nocmd) {
    if (millis() - t > 300) {
      Stop();
      Serial.print("NoCmd:");
      Serial.println(millis()-t);
      nocmd = true;
    }
  }
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
long lastmove;
bool DoMove(long value) {
  switch(value) {
      case 0xFFFFFFFF: // repeat
        DoMove(lastmove);
        break;
      case 0xFF629D: // Mode
        automove = !automove;
        break;
      case 0xFF18E7: // 2
        Serial.println("MoveFW");
        MoveForward();
        lastmove = value;
        break;
      case 0xFF30CF: // 1
      case 0xFF10EF: // 4
        TurnLeft();
        Serial.println("MoveLT");
        lastmove = value;
        break;
      case 0xFF38C7: // 5
        MoveBackward();
        Serial.println("MOVEBW");
        lastmove = value;
        break;
      case 0xFF7A85: // 3
      case 0xFF5AA5: // 6
        Serial.println("MoveRT");
        lastmove = value; 
        TurnRight();
        break;
      default:
        lastmove = 0;
        return false;
    }
    //delay(100);
    return true;
}
#endif

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

#ifdef USE_BLUETOOTH
void BTControl() {
  // THROTTLE AND STEERING CONTROL
  // throttle values after subtracting 49:
  //     50 = max forward throttle
  //     0 = no throttole
  //     -49 = max reverse throttle
  // steering values after subtracting 49:
  //     50 = max right
  //     0 = straight
  //     -49 = max left
  int throttle = phone.getThrottle() - 49;
  int steering = phone.getSteering() - 49;

  if (throttle == 0) {
    // If throttle is zero, don't move.
    Stop();
    return;
  }

  // Determine forwards or backwards.
  if (throttle > 20) {
    // Forward
    MoveForward();
  }
  else {
    // Backward
    MoveBackward();
  }
  
  // Map throttle to PWM range.
  int mappedSpeed = map(abs(throttle), 0, 50, MINIMUM_MOTOR_SPEED, 255);
  // Map steering to PWM range.
  int reducedSpeed = map(abs(steering), 0, 50, mappedSpeed, MINIMUM_MOTOR_SPEED);

  int leftMotorSpeed, rightMotorSpeed;
  if (steering > 0) {
    // Turn Right: reduce right motor speed
    leftMotorSpeed = mappedSpeed;
    rightMotorSpeed = reducedSpeed;
  }
  else {
    // Turn Left: reduce left motor speed
    leftMotorSpeed = reducedSpeed;
    rightMotorSpeed = mappedSpeed;
  }

  SetLeftSpeed(leftMotorSpeed);
  SetRightSpeed(rightMotorSpeed);

  // Print Debug Info
  Serial.print("throttle: "); Serial.print(throttle);
  Serial.print("\tsteering: "); Serial.print(steering);
  Serial.print("\tmappedSpeed: "); Serial.print(mappedSpeed);
  Serial.print("\treducedSpeed: "); Serial.print(reducedSpeed);
  Serial.print("\tleftMotorSpeed: "); Serial.print(leftMotorSpeed);
  Serial.print("\trightMotorSpeed: "); Serial.println(rightMotorSpeed);
}
#endif
