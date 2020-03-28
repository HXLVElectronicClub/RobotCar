#include "Move.h"
#include "arduino.h"
int LEFT,RIGHT;
int LEFTR, RIGHTR;
int ENABLEL, ENABLER;
int L239D_DRIVE = 0;

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
    SetRightSpeed(230);
  }
}

// v=1 turn forward
// v=0 stop
// v=-1 turn backward
void LeftWheel(int v) {
  if (!L239D_DRIVE) {
    if (v==0 || v==-1) {
      digitalWrite(LEFT, 1);
    } else if (v==1) {
      digitalWrite(LEFT, 0);
    }
  } else {
    if (v == 0) {
      digitalWrite(LEFT, 0);
      digitalWrite(LEFTR,0);
    } else if (v==1) {
      digitalWrite(LEFT, 1);
      digitalWrite(LEFTR,0);
    } else if (v==-1) {
      digitalWrite(LEFT, 0);
      digitalWrite(LEFTR,1);
    }
  }
}

void RightWheel(int v) {
  if (!L239D_DRIVE) {
    if (v==0 || v==-1) {
      digitalWrite(RIGHT, 1);
    } else if (v==1) {
      digitalWrite(RIGHT, 0);
    }
  } else {
    if (v == 0) {
      digitalWrite(RIGHT, 0);
      digitalWrite(RIGHTR,0);
    } else if (v==1) {
      digitalWrite(RIGHT, 1);
      digitalWrite(RIGHTR,0);
    } else if (v==-1) {
      digitalWrite(RIGHT, 0);
      digitalWrite(RIGHTR,1);
    }
  }
}

void MoveForward(int t) {
  LeftWheel(1);
  RightWheel(1);
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
    if (t>0) {
      delay(t);
      Stop();
    }
  }
}

void Stop() {
  LeftWheel(0);
  RightWheel(0);
}

void TurnRight(int t) {
  LeftWheel(1);
  RightWheel(-1);
}

void TurnLeft(int t) {
  LeftWheel(-1);
  RightWheel(1);
}

void SetLeftSpeed(int s) {
  analogWrite(ENABLEL,s);
}

void SetRightSpeed(int s) {
  analogWrite(ENABLER,s);
}
