/*
 * Maze Solver - Diagonal Sensor Configuration
 * SENSOR POSITIONS: 
 *   - LEFT  = NORTH-WEST (45° left-front diagonal)
 *   - RIGHT = NORTH-EAST (45° right-front diagonal)
 *   - FRONT = NORTH (straight ahead)
 * 
 * UPDATES:
 * 1. Adjusted distance calculations for diagonal sensor angles
 * 2. Modified PID for diagonal wall sensing
 * 3. Improved corner detection with angled sensors
 * 4. Goal Detection (>40cm all sides)
 */

#include <NewPing.h>

// ======================= PIN DEFINITIONS =======================

#define FRONT_TRIG_PIN 14
#define FRONT_ECHO_PIN 35
#define LEFT_TRIG_PIN 18      // NW Sensor
#define LEFT_ECHO_PIN 13
#define RIGHT_TRIG_PIN 16     // NE Sensor
#define RIGHT_ECHO_PIN 34

#define MAX_DISTANCE 100 
#define SENSOR_SAMPLES 3

NewPing sonarFront(FRONT_TRIG_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarLeft(LEFT_TRIG_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);    // NW
NewPing sonarRight(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE); // NE

// L298N Motor Driver
#define MOTOR_IN1 23
#define MOTOR_IN2 22
#define MOTOR_ENA 25 
#define MOTOR_IN3 21
#define MOTOR_IN4 19
#define MOTOR_ENB 26 

#define STATUS_LED 2

// ======================= TUNING CONSTANTS =======================

// Motor Speed
#define BASE_SPEED 130      
#define TURN_SPEED 140      
#define MAX_SPEED 160       

// ======================= DIAGONAL SENSOR ADJUSTMENTS =======================
// Since sensors are at 45°, measured distance is longer than perpendicular distance
// Perpendicular = Measured * cos(45°) ≈ Measured * 0.707
// We compensate by using LARGER thresholds (measured distances are bigger)

#define DIAGONAL_FACTOR 1.414  // 1/cos(45°) - multiply perpendicular desired by this

// Wall Following Thresholds (adjusted for diagonal sensors)
// These are the RAW sensor readings, which are longer due to angle
#define DESIRED_WALL_DIST 10.0   // Perpendicular ~7cm, but measured diagonal ~10cm
#define FRONT_THRESHOLD 12.0     // Front sensor still straight, no change needed
#define NO_WALL_THRESHOLD 40.0   // Diagonal sees further, increase threshold

// Corner Detection - diagonal sensors excel at this!
#define CORNER_DETECT_THRESHOLD 25.0  // When diagonal sees opening ahead

// Goal Threshold - INCREASED for diagonal sensors!
// Diagonal sensors read ~1.4x further, so we need higher threshold
// Also use perpendicular check to be extra sure
#define GOAL_THRESHOLD_FRONT 50.0      // Front sensor (straight)
#define GOAL_THRESHOLD_DIAGONAL 70.0   // NW/NE sensors (diagonal reads longer) 

// Time-Based Turn Durations
#define TURN_90_TIME 350        
#define TURN_180_TIME 800      
#define FORWARD_AFTER_GAP 250   // Slightly less - diagonal sees gaps earlier && If it misses openings: Decrease FORWARD_AFTER_GAP
#define FORWARD_INTO_CELL 350   

// --- PID SETTINGS FOR DIAGONAL SENSORS ---
// Diagonal sensors give smoother readings and earlier wall detection
// This allows for gentler corrections
float Kp = 2.0;      // Moderate - diagonal gives good feedback && If robot over-corrects: Lower Kp to 1.5
float Ki = 0.0;      // No integral for now && If robot wobbles: Increase Kd to 1.5
float Kd = 1.0;      // Damping for smooth corrections && 

// DEADBAND - Diagonal sensors have more tolerance
#define DEADBAND 2.0  
#define MIN_SPEED 100

// ======================= GLOBAL VARIABLES =======================

float prevError = 0;
float integralError = 0;

// Stuck detection variables
float lastFront = 0, lastLeft = 0, lastRight = 0;
int stuckCounter = 0;
#define STUCK_THRESHOLD 3       
#define STUCK_TOLERANCE 2.0     
#define BACKUP_TIME 400         

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
    // This helps with wall distance calculations
    float dLeftPerp = dLeftDiag * 0.707;   // Approximate perpendicular to left wall
    float dRightPerp = dRightDiag * 0.707; // Approximate perpendicular to right wall
    
    Serial.printf("F:%.1f NW:%.1f(~%.1f) NE:%.1f(~%.1f) | ", 
                  dFront, dLeftDiag, dLeftPerp, dRightDiag, dRightPerp);

    // ========================================================
    // PRIORITY 0: GOAL REACHED
    // Uses BOTH diagonal raw readings AND perpendicular approximations
    // to prevent false triggers
    // ========================================================
    bool frontOpen = (dFront > GOAL_THRESHOLD_FRONT);
    bool leftOpen = (dLeftDiag > GOAL_THRESHOLD_DIAGONAL) && (dLeftPerp > 35.0);  // ~50cm perpendicular
    bool rightOpen = (dRightDiag > GOAL_THRESHOLD_DIAGONAL) && (dRightPerp > 35.0);
    
    if (frontOpen && leftOpen && rightOpen) {
        Serial.println("\n\n!!! GOAL REACHED !!!");
        Serial.printf("Final: F=%.1f, NW=%.1f(perp:%.1f), NE=%.1f(perp:%.1f)\n",
                      dFront, dLeftDiag, dLeftPerp, dRightDiag, dRightPerp);
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
        
        // 1. Back up first to clear the obstacle
        setMotors(-BASE_SPEED, -BASE_SPEED);
        delay(BACKUP_TIME);
        stopMotors();
        delay(200);
        
        // 2. Re-scan after backing up
        float newLeft = readDistanceLeft();
        float newRight = readDistanceRight();
        
        Serial.printf("Recovery Scan -> NW:%.1f NE:%.1f\n", newLeft, newRight);
        
        // 3. Turn towards the most open space
        if (newLeft > newRight) {
             Serial.println("Recovery: NW more open -> Turn LEFT");
             turnLeft();
        } else {
             Serial.println("Recovery: NE more open -> Turn RIGHT");
             turnRight();
        }
        
        prevError = 0;
        integralError = 0;
        return;
    }

    // ========================================================
    // PRIORITY 2: Left Opening Detection (Diagonal Advantage!)
    // Diagonal sensors can detect openings BEFORE reaching them
    // ========================================================
    if (dLeftDiag > NO_WALL_THRESHOLD && dLeftPerp > CORNER_DETECT_THRESHOLD) {
        Serial.println("NW SEES OPENING! Preparing Left Turn.");
        stopMotors();
        delay(100);
        
        // With diagonal sensors, we detect gaps earlier
        // Drive forward a bit less before turning
        driveForward(BASE_SPEED);
        delay(FORWARD_AFTER_GAP);
        turnLeft();
        driveForward(BASE_SPEED);
        delay(FORWARD_INTO_CELL);
        
        prevError = 0; 
        integralError = 0;
        return;
    }

    // ========================================================
    // PRIORITY 3: Front Blocked (Turn Right or U-Turn)
    // ========================================================
    if (dFront < FRONT_THRESHOLD) {
        Serial.println("FRONT BLOCKED!");
        stopMotors();
        delay(100);
        
        // Check diagonal sensors for best escape route
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
        prevError = 0;
        integralError = 0;
        return;
    }

    // ========================================================
    // PRIORITY 4: Diagonal PID Wall Follow
    // Use the DIAGONAL readings directly for smoother control
    // ========================================================
    pidWallFollowDiagonal(dLeftDiag, dRightDiag, dFront);
}

// ======================= DIAGONAL PID CONTROL =======================

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
    if (abs(error) < DEADBAND) {
        error = 0;
        Serial.print("(DB) ");
    }

    // Constrain error
    error = constrain(error, -8.0, 8.0);  // Wider range for diagonal sensors

    // --- PID CALCULATION ---
    float P = Kp * error;
    integralError += error;
    integralError = constrain(integralError, -20.0, 20.0);
    float I = Ki * integralError;
    float D = Kd * (error - prevError);
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

// ======================= TURNS =======================

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

// ======================= MOTORS & SENSORS =======================

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
