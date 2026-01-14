#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ======================= PIN DEFINITIONS =======================

#define FRONT_TRIG_PIN 14
#define FRONT_ECHO_PIN 35
#define LEFT_TRIG_PIN 18      // NW Sensor
#define LEFT_ECHO_PIN 13
#define RIGHT_TRIG_PIN 16     // NE Sensor
#define RIGHT_ECHO_PIN 34

#define MOTOR_IN1 23
#define MOTOR_IN2 22
#define MOTOR_ENA 25 
#define MOTOR_IN3 21
#define MOTOR_IN4 19
#define MOTOR_ENB 26 

#define STATUS_LED 2

// ======================= TUNING CONSTANTS =======================

#define MAX_DISTANCE 100 
#define SENSOR_SAMPLES 3

// Motor Speed
#define BASE_SPEED 130      
#define TURN_SPEED 140      
#define MAX_SPEED 160
#define MIN_SPEED 100       

// ======================= DIAGONAL SENSOR ADJUSTMENTS =======================
#define DIAGONAL_FACTOR 1.414  // 1/cos(45°)

// Wall Following Thresholds
#define DESIRED_WALL_DIST 10.0   
#define FRONT_THRESHOLD 12.0     
#define NO_WALL_THRESHOLD 40.0   

// Corner Detection
#define CORNER_DETECT_THRESHOLD 25.0  

// Goal Threshold
#define GOAL_THRESHOLD_FRONT 50.0      
#define GOAL_THRESHOLD_DIAGONAL 70.0   

// Time-Based Turn Durations
#define TURN_90_TIME 350        
#define TURN_180_TIME 800      
#define FORWARD_AFTER_GAP 250   
#define FORWARD_INTO_CELL 350   

// PID SETTINGS
#define PID_KP 2.0      
#define PID_KI 0.0      
#define PID_KD 1.0      
#define PID_DEADBAND 2.0  

// STUCK DETECTION
#define STUCK_THRESHOLD 3       
#define STUCK_TOLERANCE 2.0     
#define BACKUP_TIME 400         

#endif
