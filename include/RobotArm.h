#ifndef ROBOTARM_H
#define ROBOTARM_H

#include <Servo.h>
#include "Kinematics.h"

class RobotArm {
  private:
    Kinematics* solver;
    Servo servoShoulder;
    Servo servoElbow;
    Servo servoWrist;
    Servo servoGripper;

    // --- CALIBRATION OFFSETS ---
    // Change these values when you physically mount the servo horns.
    // If the math says "0 degrees" (straight out), but your servo needs
    // to be at "90" to physically point straight out, set the offset to 90.
    float shoulderOffset = 90.0; 
    float elbowOffset = 0.0;     
    float wristOffset = 90.0;    

  public:
    // Pass in the kinematics solver and attach pins
    RobotArm(Kinematics* mathEngine, int pinS, int pinE, int pinW, int pinG) {
        solver = mathEngine;
        servoShoulder.attach(pinS);
        servoElbow.attach(pinE);
        servoWrist.attach(pinW);
        servoGripper.attach(pinG);
    }

    // Move the arm to a specific X/Y coordinate with a specific approach angle
    bool moveToTarget(float x, float y, float phi) {
        TargetPose target = {x, y, phi};
        JointAngles result = solver->calculateAngles(target);

        if (result.reachable) {
            // Apply physical offsets to the raw mathematical angles
            float physShoulder = result.shoulder + shoulderOffset;
            float physElbow = result.elbow + elbowOffset;
            float physWrist = result.wrist + wristOffset;

            // Constrain bounds to strictly 0 to 180 to prevent servo damage
            physShoulder = constrain(physShoulder, 0, 180);
            physElbow = constrain(physElbow, 0, 180);
            physWrist = constrain(physWrist, 0, 180);

            // Write safe, mapped angles to hardware
            servoShoulder.write(physShoulder);
            servoElbow.write(physElbow);
            servoWrist.write(physWrist);
            
            return true;
        } else {
            Serial.println("Error: Target coordinates are out of physical reach!");
            return false;
        }
    }

    // Simple open/close command for the claw
    void setGripper(bool open) {
        if (open) {
            servoGripper.write(90); // Adjust this angle for your specific claw open state
        } else {
            servoGripper.write(10); // Adjust this angle for your specific claw closed state
        }
    }
};

#endif