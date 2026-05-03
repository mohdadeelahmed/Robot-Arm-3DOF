#include <Arduino.h>
#include "Kinematics.h"
#include "RobotArm.h"

// 1. Enter your exact 3D model measurements here in millimeters
float link1_length = 120.0; // Base to Elbow
float link2_length = 100.0; // Elbow to Wrist
float link3_length = 50.0;  // Wrist to tip of Gripper

// 2. Initialize the math engine
Kinematics mathEngine(link1_length, link2_length, link3_length);

// 3. Initialize the arm (passing the math engine and the servo pins 9, 10, 11, 12)
RobotArm myArm(&mathEngine, 9, 10, 11, 12);

void setup() {
    Serial.begin(9600);
    
    // Example movement: 
    // Move to X: 150mm, Y: 100mm, approaching flat horizontally (0 degrees)
    myArm.moveToTarget(150.0, 100.0, 0.0);
    
    delay(1000);
    myArm.setGripper(false); // Close claw
}

void loop() {
    // You can add your sequence logic here
}