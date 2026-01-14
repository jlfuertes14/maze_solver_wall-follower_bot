#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

void initMotors();
void setMotors(int L, int R);
void stopMotors();
void driveForward(int speed);
void turnLeft();
void turnRight();
void turnAround();

#endif
