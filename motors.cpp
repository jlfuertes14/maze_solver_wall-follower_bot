#include "motors.h"
#include "config.h"

void initMotors() {
    pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT); pinMode(MOTOR_IN4, OUTPUT);
    pinMode(MOTOR_ENA, OUTPUT); pinMode(MOTOR_ENB, OUTPUT);
}

void setMotors(int L, int R) {
    if (L >= 0) { 
        digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW); 
    } else { 
        digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, HIGH); 
        L = -L; 
    }
    analogWrite(MOTOR_ENA, constrain(L, 0, 255));

    if (R >= 0) { 
        digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW); 
    } else { 
        digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, HIGH); 
        R = -R; 
    }
    analogWrite(MOTOR_ENB, constrain(R, 0, 255));
}

void stopMotors() { setMotors(0, 0); }

void driveForward(int speed) { setMotors(speed, speed); }

void turnLeft() {
    Serial.println(">>> Turning LEFT");
    setMotors(-TURN_SPEED, TURN_SPEED); 
    delay(TURN_90_TIME);
    stopMotors();
    delay(100);
}

void turnRight() {
    Serial.println(">>> Turning RIGHT");
    setMotors(TURN_SPEED, -TURN_SPEED);
    delay(TURN_90_TIME);
    stopMotors();
    delay(100);
}

void turnAround() {
    Serial.println(">>> Turning 180");
    setMotors(TURN_SPEED, -TURN_SPEED);
    delay(TURN_180_TIME);
    stopMotors();
    delay(100);
}
