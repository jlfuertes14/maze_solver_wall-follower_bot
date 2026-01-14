#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <NewPing.h>

void initSensors();
float readDistanceFront();
float readDistanceLeft();
float readDistanceRight();

#endif
