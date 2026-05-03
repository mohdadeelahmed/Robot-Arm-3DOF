#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include "Position.h"

class Kinematics {
  private:
    float L1; // Shoulder to elbow length
    float L2; // Elbow to wrist length
    float L3; // Wrist to gripper tip length

  public:
    // Constructor where you input your physical 3D model dimensions
    Kinematics(float link1_mm, float link2_mm, float link3_mm) {
        L1 = link1_mm;
        L2 = link2_mm;
        L3 = link3_mm;
    }

    // The main Inverse Kinematics solver
    JointAngles calculateAngles(TargetPose target) {
        JointAngles angles;
        angles.reachable = true;

        // Convert target angle to radians for math functions
        float phiRad = target.phi * (PI / 180.0);

        // Step 1: Find the wrist position
        float wx = target.x - L3 * cos(phiRad);
        float wy = target.y - L3 * sin(phiRad);

        // Step 2: Calculate elbow angle (Law of Cosines)
        float cosTheta2 = (wx*wx + wy*wy - L1*L1 - L2*L2) / (2.0 * L1 * L2);
        
        // Safety check: if cosTheta2 is outside [-1, 1], the point is out of reach
        if (cosTheta2 > 1.0 || cosTheta2 < -1.0) {
            angles.reachable = false;
            cosTheta2 = constrain(cosTheta2, -1.0, 1.0); // Prevent math crash
        }
        
        float theta2Rad = acos(cosTheta2); // This gives the "elbow down" solution

        // Step 3: Calculate shoulder angle
        float k1 = L1 + L2 * cos(theta2Rad);
        float k2 = L2 * sin(theta2Rad);
        float theta1Rad = atan2(wy, wx) - atan2(k2, k1);

        // Step 4: Calculate wrist angle
        float theta3Rad = phiRad - theta1Rad - theta2Rad;

        // Convert everything back to degrees for the servos
        angles.shoulder = theta1Rad * (180.0 / PI);
        angles.elbow = theta2Rad * (180.0 / PI);
        angles.wrist = theta3Rad * (180.0 / PI);

        return angles;
    }
};

#endif