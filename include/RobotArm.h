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
            // Write to servos. You might need to map or offset these values 
            // depending on how you mount the physical servo horns.
            servoShoulder.write(result.shoulder);
            servoElbow.write(result.elbow);
            servoWrist.write(result.wrist);
            return true;
        } else {
            Serial.println("Error: Target coordinates are out of physical reach!");
            return false;
        }
    }

    // Simple open/close command for the claw
    void setGripper(bool open) {
        if (open) {
            servoGripper.write(90); // Adjust this angle for your specific claw
        } else {
            servoGripper.write(10); // Adjust this angle for your specific claw
        }
    }
};

#endif