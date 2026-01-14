#include "pid.h"
#include "config.h"
#include "motors.h"

// PID Variables
float prevError = 0;
float integralError = 0;

void pidWallFollowDiagonal(float leftDiag, float rightDiag, float frontDist) {
    float error = 0;
    
    // Wall detection using diagonal thresholds
    bool leftWall = (leftDiag < NO_WALL_THRESHOLD);
    bool rightWall = (rightDiag < NO_WALL_THRESHOLD);
    
    // --- CALCULATE ERROR ---
    // For diagonal sensors, we use the readings directly
    // Larger left reading = we're angled away from left wall (or no wall)
    // Smaller left reading = we're angled toward left wall
    
    if (leftWall && rightWall) {
        // Both walls visible - center between them
        // If leftDiag > rightDiag, we're closer to right wall -> turn left
        error = leftDiag - rightDiag;
        Serial.print("CORRIDOR ");
    }
    else if (leftWall) {
        // Only NW sensor sees wall - maintain distance
        // If leftDiag > DESIRED, we're too far -> turn left (negative error)
        // If leftDiag < DESIRED, we're too close -> turn right (positive error)
        error = DESIRED_WALL_DIST - leftDiag;  // Inverted: closer to wall = turn away
        Serial.print("FOLLOW-L ");
    }
    else if (rightWall) {
        // Only NE sensor sees wall - maintain distance
        error = rightDiag - DESIRED_WALL_DIST;  // Closer to right = turn left
        Serial.print("FOLLOW-R ");
    }
    else {
        // No walls - try to go straight, maybe slight left bias for left-hand rule
        error = 0;
        Serial.print("OPEN ");
    }

    // --- APPLY DEADBAND ---
    if (abs(error) < PID_DEADBAND) {
        error = 0;
        Serial.print("(DB) ");
    }

    // Constrain error
    error = constrain(error, -8.0, 8.0);  // Wider range for diagonal sensors

    // --- PID CALCULATION ---
    float P = PID_KP * error;
    integralError += error;
    integralError = constrain(integralError, -20.0, 20.0);
    float I = PID_KI * integralError;
    float D = PID_KD * (error - prevError);
    float correction = P + I + D;
    
    prevError = error;

    // --- PREDICTIVE STEERING (Diagonal Advantage!) ---
    // If front is getting closer, pre-emptively start turning
    if (frontDist < 25 && frontDist > FRONT_THRESHOLD) {
        // Front obstacle approaching - bias toward more open side
        if (leftDiag > rightDiag) {
            correction -= 5;  // Slight left bias
            Serial.print("[PRE-L] ");
        } else {
            correction += 5;  // Slight right bias
            Serial.print("[PRE-R] ");
        }
    }

    // --- MOTOR MIXING ---
    // Positive correction -> Turn Left (Left slower, Right faster)
    // Negative correction -> Turn Right (Left faster, Right slower)
    
    int leftSpeed = BASE_SPEED - correction;
    int rightSpeed = BASE_SPEED + correction;

    leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

    setMotors(leftSpeed, rightSpeed);
    Serial.printf("Err:%.1f Cor:%.1f L:%d R:%d\n", error, correction, leftSpeed, rightSpeed);
}
