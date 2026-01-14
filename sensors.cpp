#include "sensors.h"
#include "config.h"

NewPing sonarFront(FRONT_TRIG_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarLeft(LEFT_TRIG_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);    // NW
NewPing sonarRight(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE); // NE

void initSensors() {
    pinMode(FRONT_TRIG_PIN, OUTPUT); pinMode(FRONT_ECHO_PIN, INPUT);
    pinMode(LEFT_TRIG_PIN, OUTPUT); pinMode(LEFT_ECHO_PIN, INPUT);
    pinMode(RIGHT_TRIG_PIN, OUTPUT); pinMode(RIGHT_ECHO_PIN, INPUT);
}

float readDistanceFront() {
    unsigned int d = sonarFront.ping_median(SENSOR_SAMPLES);
    return (d == 0) ? 999.0 : sonarFront.convert_cm(d);
}

float readDistanceLeft() {  // NW Sensor
    unsigned int d = sonarLeft.ping_median(SENSOR_SAMPLES);
    return (d == 0) ? 999.0 : sonarLeft.convert_cm(d);
}

float readDistanceRight() { // NE Sensor
    unsigned int d = sonarRight.ping_median(SENSOR_SAMPLES);
    return (d == 0) ? 999.0 : sonarRight.convert_cm(d);
}
