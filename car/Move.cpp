#include "Move.h"
#include "arduino.h"
int LEFT,RIGHT;
int LEFTR, RIGHTR;
int ENABLEL, ENABLER;
int L239D_DRIVE = 0;
int USE_74HC595;
int IN,CLK,LATCH;
float leftspeedratio = 1;
float rightspeedratio = 1;
bool moving = true;;
bool debug_move = false;
int LeftStatus=0,RightStatus=0;

void MotorPins(int left, int right) {
  LEFT = left;
  RIGHT = right;
  pinMode(LEFT, OUTPUT);
  pinMode(RIGHT, OUTPUT);
  Stop();
}

void MotorPins(int left, int leftr, int right, int rightr, int enablel, int enabler) {
  L239D_DRIVE=1;
  LEFT = left;
  LEFTR = leftr;
  RIGHT = right;
  RIGHTR = rightr;
  pinMode(LEFT, OUTPUT);
  pinMode(RIGHT, OUTPUT);
  pinMode(LEFTR, OUTPUT);
  pinMode(RIGHTR, OUTPUT);
  Stop();
  if (enablel!=0) {
    ENABLEL = enablel;
    pinMode(ENABLEL, OUTPUT);
    SetLeftSpeed(255);
  }
  if (enabler!=0) {
    ENABLER = enabler;
    pinMode(ENABLER, OUTPUT);
    SetRightSpeed(255);
  }
}

void MotorPins_shift(int in, int clk, int latch, int enablel, int enabler) {
  L239D_DRIVE=1;
  USE_74HC595=1;
  IN = in;
  CLK = clk;
  LATCH = latch;
  pinMode(IN, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(LATCH, OUTPUT);
  if (enablel!=0) {
    ENABLEL = enablel;
    SetLeftSpeed(255);
  }
  if (enabler!=0) {
    ENABLER = enabler;
    SetRightSpeed(255);
  }  
  Stop();
}

void RnWDigSig(int sig, int value) {
  if (digitalRead(sig) != value) {
    digitalWrite(sig,value);
  }
}

// v=1 turn forward
// v=0 stop
// v=-1 turn backward
void LeftWheel(int v) {
  LeftStatus = v;
}

void RightWheel(int v) {
  RightStatus = v;
}

void MoveForward(int t) {
  LeftWheel(1);
  RightWheel(1);
  Drive();
  moving = true;
  if (t > 0) {
    delay(t);
    Stop();
  }
}

void MoveBackward(int t) {
  if (!L239D_DRIVE) {
    Stop();
  } else {
    LeftWheel(-1);
    RightWheel(-1);
    Drive();
    moving = true;
    if (t>0) {
      delay(t);
      Stop();
    }
  }
}

void Stop() {
  if (debug_move) Serial.println("Stop called");
  if (moving) {
    LeftWheel(0);
    RightWheel(0);
    Drive();
    moving = false;
  }
}

void TurnRight(int t) {
  LeftWheel(1);
  RightWheel(-1);
  Drive();
  moving = true;
}

void TurnLeft(int t) {
  LeftWheel(-1);
  RightWheel(1);
  Drive();
  moving = true;
}

void SetLeftSpeed(int s) {
  analogWrite(ENABLEL,s*leftspeedratio);
}

void SetRightSpeed(int s) {
  analogWrite(ENABLER,s*rightspeedratio);
}

void SetSpeedRatio(float l, float r) {
  leftspeedratio = l;
  rightspeedratio = r;
}

void Drive() {
  static byte lastShift = 0xff;
  if (!USE_74HC595) {
    if (!L239D_DRIVE){
      RnWDigSig(LEFT, (LeftStatus  == 0 || LeftStatus  == -1)?1:0);
      RnWDigSig(RIGHT,(RightStatus == 0 || RightStatus == -1)?1:0);
    } else {
      RnWDigSig(LEFT , (LeftStatus  == 0 || LeftStatus  == -1)?0:1);
      RnWDigSig(LEFTR, (LeftStatus  == 0 || LeftStatus  ==  1)?0:1);
      RnWDigSig(RIGHT , (RightStatus  == 0 || RightStatus  == -1)?0:1);
      RnWDigSig(RIGHTR, (RightStatus  == 0 || RightStatus  ==  1)?0:1);
    }
  } else {
    byte bitsToSend = 0;
    bitWrite(bitsToSend, 1, (LeftStatus==0||LeftStatus==-1)?0:1); // L
    bitWrite(bitsToSend, 0, (LeftStatus==0||LeftStatus== 1)?0:1); // L_R
    bitWrite(bitsToSend, 3, (RightStatus==0||RightStatus==-1)?0:1); // R
    bitWrite(bitsToSend, 2, (RightStatus==0||RightStatus== 1)?0:1); // R_R
    if (bitsToSend != lastShift) {
      if (debug_move) {Serial.print("Shift out: 0b");Serial.println(bitsToSend,BIN);}
      digitalWrite(LATCH, LOW);
      shiftOut(IN,CLK,MSBFIRST,bitsToSend);
      digitalWrite(LATCH,HIGH);
      lastShift = bitsToSend;
      delay(10);
    }
  }
}
