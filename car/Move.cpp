#include "Move.h"
#include "arduino.h"
int LEFT,RIGHT;
int LEFTR, RIGHTR;
int ENABLEL, ENABLER;
int L239D_DRIVE = 0;
float leftspeedratio = 1;
float rightspeedratio = 1;
bool moving = false;;
bool debug_move = false;

void MotorPins(int left, int right) {
  LEFT = left;
  RIGHT = right;
  Stop();
}

void MotorPins(int left, int leftr, int right, int rightr, int enablel, int enabler) {
  L239D_DRIVE=1;
  LEFT = left;
  LEFTR = leftr;
  RIGHT = right;
  RIGHTR = rightr;
  Stop();
  if (enablel!=0) {
    ENABLEL = enablel;
    SetLeftSpeed(255);
  }
  if (enabler!=0) {
    ENABLER = enabler;
    SetRightSpeed(255);
  }
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
  if (debug_move) Serial.println("Left wheel motion");
  if (!L239D_DRIVE) {
    if (v==0 || v==-1) {
      RnWDigSig(LEFT,1);
    } else if (v==1) {
      RnWDigSig(LEFT,0);
    }
  } else {
    if (v == 0) {
      RnWDigSig(LEFT,0);
      RnWDigSig(LEFTR,0);
    } else if (v==1) {
      RnWDigSig(LEFT,1);
      RnWDigSig(LEFTR,0);
    } else if (v==-1) {
      RnWDigSig(LEFT,0);
      RnWDigSig(LEFTR,1);
    }
  }
}

void RightWheel(int v) {
  if (debug_move) Serial.println("Right wheel motion");
  if (!L239D_DRIVE) {
    if (v==0 || v==-1) {
      RnWDigSig(RIGHT,1);
    } else if (v==1) {
      RnWDigSig(RIGHT,0);
    }
  } else {
    if (v == 0) {
      RnWDigSig(RIGHT,0);
      RnWDigSig(RIGHTR,0);
    } else if (v==1) {
      RnWDigSig(RIGHT,1);
      RnWDigSig(RIGHTR,0);
    } else if (v==-1) {
      RnWDigSig(RIGHT,0);
      RnWDigSig(RIGHTR,1);
    }
  }
}

void MoveForward(int t) {
  LeftWheel(1);
  RightWheel(1);
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
    moving = false;
  }
}

void TurnRight(int t) {
  LeftWheel(1);
  RightWheel(-1);
  moving = true;
}

void TurnLeft(int t) {
  LeftWheel(-1);
  RightWheel(1);
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
