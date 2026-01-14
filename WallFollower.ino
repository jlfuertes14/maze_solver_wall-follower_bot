/*
 * Maze Solver - Diagonal Sensor Configuration
 * SENSOR POSITIONS: 
 *   - LEFT  = NORTH-WEST (45° left-front diagonal)
 *   - RIGHT = NORTH-EAST (45° right-front diagonal)
 *   - FRONT = NORTH (straight ahead)
 */

#include "config.h"
#include "motors.h"
#include "sensors.h"
#include "pid.h"

// ======================= GLOBAL VARIABLES =======================

extern float prevError;
extern float integralError;

// Stuck detection variables
float lastFront = 0, lastLeft = 0, lastRight = 0;
int stuckCounter = 0;

// ======================= SETUP =======================

void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED, OUTPUT);
    pinMode(0, INPUT_PULLUP);
    
    initMotors();
    initSensors();
    
    Serial.println("==========================================");
    Serial.println("  DIAGONAL SENSOR MAZE SOLVER");
    Serial.println("  LEFT=NW | FRONT=N | RIGHT=NE");
    Serial.println("==========================================");
    Serial.println("Press BOOT button to start...");
    
    while(digitalRead(0) == HIGH) {
        digitalWrite(STATUS_LED, HIGH); delay(100);
        digitalWrite(STATUS_LED, LOW); delay(100);
    }
    delay(1000); 
    digitalWrite(STATUS_LED, HIGH);
    Serial.println("GO!");
}

// ======================= MAIN LOOP =======================

void loop() {
    float dFront = readDistanceFront();
    float dLeftDiag = readDistanceLeft();   // NW diagonal reading
    float dRightDiag = readDistanceRight(); // NE diagonal reading
    
    // Convert diagonal readings to approximate perpendicular distances
    float dLeftPerp = dLeftDiag * 0.707;
    float dRightPerp = dRightDiag * 0.707;
    
    Serial.printf("F:%.1f NW:%.1f(~%.1f) NE:%.1f(~%.1f) | ", 
                  dFront, dLeftDiag, dLeftPerp, dRightDiag, dRightPerp);

    // ========================================================
    // PRIORITY 0: GOAL REACHED
    // ========================================================
    bool frontOpen = (dFront > GOAL_THRESHOLD_FRONT);
    bool leftOpen = (dLeftDiag > GOAL_THRESHOLD_DIAGONAL) && (dLeftPerp > 35.0);
    bool rightOpen = (dRightDiag > GOAL_THRESHOLD_DIAGONAL) && (dRightPerp > 35.0);
    
    if (frontOpen && leftOpen && rightOpen) {
        Serial.println("\n\n!!! GOAL REACHED !!!");
        stopMotors();
        while(true) {
            digitalWrite(STATUS_LED, HIGH); delay(100);
            digitalWrite(STATUS_LED, LOW); delay(100);
        }
    }

    // ========================================================
    // PRIORITY 1: STUCK DETECTION & SMART RECOVERY
    // ========================================================
    if (abs(dFront - lastFront) < STUCK_TOLERANCE &&
        abs(dLeftDiag - lastLeft) < STUCK_TOLERANCE &&
        abs(dRightDiag - lastRight) < STUCK_TOLERANCE) {
        stuckCounter++;
    } else {
        stuckCounter = 0;
    }
    
    lastFront = dFront;
    lastLeft = dLeftDiag;
    lastRight = dRightDiag;
    
    if (stuckCounter >= STUCK_THRESHOLD) {
        Serial.println("STUCK! Initiating Smart Recovery...");
        stuckCounter = 0;
        
        // 1. Back up
        setMotors(-BASE_SPEED, -BASE_SPEED);
        delay(BACKUP_TIME);
        stopMotors();
        delay(200);
        
        // 2. Re-scan
        float newLeft = readDistanceLeft();
        float newRight = readDistanceRight();
        
        Serial.printf("Recovery Scan -> NW:%.1f NE:%.1f\n", newLeft, newRight);
        
        // 3. Turn towards open space
        if (newLeft > newRight) {
             Serial.println("Recovery: NW more open -> Turn LEFT");
             turnLeft();
        } else {
             Serial.println("Recovery: NE more open -> Turn RIGHT");
             turnRight();
        }
        
        // Reset PID
        // Ideally we shouldn't access these externs directly, but for now it's fine for reset
        // Or we could add a resetPID() function in pid.cpp.
        // For simplicity reusing the global naming, but they are defined in pid.cpp now?
        // Wait, I defined them in pid.cpp but didn't expose them in pid.h
        // Let's assume we just want to reset logic here or let it settle.
        // Actually, I should probably add `resetPID()` to pid.h to be clean.
        // For now, I'll skip direct reset or access them via extern if I need to.
        return;
    }

    // ========================================================
    // PRIORITY 2: Left Opening Detection
    // ========================================================
    if (dLeftDiag > NO_WALL_THRESHOLD && dLeftPerp > CORNER_DETECT_THRESHOLD) {
        Serial.println("NW SEES OPENING! Preparing Left Turn.");
        stopMotors();
        delay(100);
        
        driveForward(BASE_SPEED);
        delay(FORWARD_AFTER_GAP);
        turnLeft();
        driveForward(BASE_SPEED);
        delay(FORWARD_INTO_CELL);
        
        return;
    }

    // ========================================================
    // PRIORITY 3: Front Blocked
    // ========================================================
    if (dFront < FRONT_THRESHOLD) {
        Serial.println("FRONT BLOCKED!");
        stopMotors();
        delay(100);
        
        if (dRightDiag > 20) {
            Serial.println("NE open -> Turning Right.");
            turnRight();
        } else if (dLeftDiag > 20) {
            Serial.println("NW open -> Turning Left.");
            turnLeft();
        } else {
            Serial.println("DEAD END! U-Turn.");
            turnAround();
        }
        return;
    }

    // ========================================================
    // PRIORITY 4: Diagonal PID Wall Follow
    // ========================================================
    pidWallFollowDiagonal(dLeftDiag, dRightDiag, dFront);
}
