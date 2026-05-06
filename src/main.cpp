#include <Arduino.h>
#include "Kinematics.h"
#include "RobotArm.h"

// 1. Updated 3D model measurements in millimeters
float link1_length = 97.37;  // Base to Elbow
float link2_length = 86.93;  // Elbow to Wrist
float link3_length = 147.90; // Wrist to tip of Gripper

Kinematics mathEngine(link1_length, link2_length, link3_length);
RobotArm myArm(&mathEngine, 9, 10, 11, 12);

// --- Millis Timer Variables ---
unsigned long previousMillis = 0;   // Stores the last time the arm moved
const long moveInterval = 2000;     // Interval at which to move (2000 ms = 2 seconds)
bool atPositionA = true;            // State tracker

void setup() {
    Serial.begin(115200); 
    
    // Initial movement to wake up the servos
    myArm.moveToTarget(150.0, 100.0, 0.0);
    myArm.setGripper(false); 
}

void loop() {
    // 1. Check the current time
    unsigned long currentMillis = millis();

    // 2. Has it been 2000ms since the last movement?
    if (currentMillis - previousMillis >= moveInterval) {
        
        // Save the last time you moved the arm
        previousMillis = currentMillis;

        // Toggle between two positions
        if (atPositionA) {
            Serial.println("Moving to Position B");
            myArm.moveToTarget(100.0, 150.0, 45.0); // Reach higher, angled up
            atPositionA = false;
        } else {
            Serial.println("Moving to Position A");
            myArm.moveToTarget(150.0, 100.0, 0.0);  // Reach lower, flat
            atPositionA = true;
        }
    }

    // 3. DO OTHER THINGS HERE!
    // Because we used millis(), the Uno reaches this part of the code 
    // thousands of times per second. You can read a sensor, update an LCD, 
    // or check a serial command right here without any lag.
}