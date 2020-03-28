#ifndef MOVE_H
#define MOVE_H
void MoveForward(int t = 0);
void MoveBackward(int t= 0);
void Stop();
void TurnLeft(int t=0);
void TurnRight(int t=0);
void MotorPins(int,int);
void MotorPins(int,int,int, int, int e1=0, int e2=0);
void SetLeftSpeed(int s);
void SetRightSpeed(int s);
#endif
