#include "Move.h"
#include "arduino.h"
int LEFT,RIGHT;

void MotorPins(int left, int right) {
  LEFT = left;
  RIGHT = right;
  Stop();
}
void MoveForward(int t) {
  digitalWrite(LEFT, 0);
  digitalWrite(RIGHT,0);
  if (t > 0) {
    delay(t);
    Stop();
  }
}

void MoveBackward(int t) {
  Stop();
  // this is not supported yet
}

void Stop() {
    digitalWrite(LEFT, 1);
    digitalWrite(RIGHT, 1);
}

void TurnRight(int t) {
  digitalWrite(LEFT, 1);
  digitalWrite(RIGHT, 0);
  if (t > 0) {
    delay(t);
    Stop();
  }
}

void TurnLeft(int t) {
  digitalWrite(LEFT, 0);
  digitalWrite(RIGHT, 1);
  if (t > 0) {
    delay(t);
    Stop();
  }
}
