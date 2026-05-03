#ifndef POSITION_H
#define POSITION_H

struct TargetPose {
    float x;       // X coordinate of the gripper tip (mm)
    float y;       // Y coordinate of the gripper tip (mm)
    float phi;     // Approach angle of the gripper (degrees)
};

struct JointAngles {
    float shoulder; // Angle for motor 1 (degrees)
    float elbow;    // Angle for motor 2 (degrees)
    float wrist;    // Angle for motor 3 (degrees)
    bool reachable; // Flag to tell you if the target is physically possible
};

#endif