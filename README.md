# 3-DOF Robotic Arm

A precision 3-axis robotic arm with inverse kinematics calculations implemented in C/C++. This project features a planar robot arm with three degrees of freedom (DOF) designed for accurate end-effector positioning and orientation control.

## Overview

This project implements a complete kinematic control system for a 3-DOF robotic arm. The system uses **kinematic decoupling** to solve inverse kinematics problems, allowing the robot to reach specified coordinates with a desired end-effector orientation.

Given a target position (X, Y) and approach angle (φ), the system calculates the required joint angles for:
- **θ₁** - Shoulder angle
- **θ₂** - Elbow angle  
- **θ₃** - Wrist angle

## System Architecture

The robotic arm consists of three rigid links connected by rotational joints.

### Hardware Specifications

| Component | Length | Notes |
|-----------|--------|-------|
| **L₁** (Shoulder to Elbow) | 97.37 mm | Measured between rotation axes |
| **L₂** (Elbow to Wrist) | 86.93 mm | Measured between rotation axes |
| **L₃** (Wrist to End Effector) | 147.90 mm | Measured between rotation axes |

## Inverse Kinematics

The system implements a four-step kinematic decoupling algorithm:

### Step 1: Kinematic Decoupling (Wrist Position)

Given target position (X, Y) and approach angle φ, calculate the wrist joint position by moving backward from the end effector:

```
Wₓ = X - L₃ cos(φ)
Wᵧ = Y - L₃ sin(φ)
```

### Step 2: Elbow Angle Calculation (θ₂)

Using the Law of Cosines on the 2-link chain (L₁ and L₂):

```
cos(θ₂) = (Wₓ² + Wᵧ² - L₁² - L₂²) / (2 L₁ L₂)
θ₂ = arccos(cos(θ₂))
```

### Step 3: Shoulder Angle Calculation (θ₁)

Calculate the angle to the wrist coordinate and subtract the inner triangle geometry:

```
k₁ = L₁ + L₂ cos(θ₂)
k₂ = L₂ sin(θ₂)
θ₁ = atan2(Wᵧ, Wₓ) - atan2(k₂, k₁)
```

With system constants substituted:
```
θ₁ = atan2(Y, X) - atan2(222.9 sin(θ₂), 97.4 + 222.9 cos(θ₂))
```

### Step 4: Wrist Angle Calculation (θ₃)

Ensure the end effector maintains the requested global approach angle φ by compensating for shoulder and elbow rotation:

```
θ = φ - θ₁ - θ₂
```

For detailed mathematical proofs and derivations, see [Kinematics.md](docs/Kinematics.md).

## Getting Started

### Prerequisites

- C++ compiler with C++11 or later support
- CMake (optional, for build automation)
- Target hardware with appropriate servo/motor control capabilities

### Building

```bash
# Clone the repository
git clone https://github.com/mohdadeelahmed/Robot-Arm-3DOF.git
cd Robot-Arm-3DOF

# Build with CMake (if available)
mkdir build
cd build
cmake ..
make

# Or compile directly with g++
g++ -std=c++11 -o robot-arm src/*.cpp
```

### Basic Usage

```cpp
#include "kinematics.h"

// Target position and approach angle
double target_x = 100.0;  // mm
double target_y = 150.0;  // mm
double approach_angle = M_PI / 4;  // 45 degrees

// Calculate joint angles
JointAngles angles = calculateInverseKinematics(
    target_x, 
    target_y, 
    approach_angle
);

// Use angles to control servos/motors
moveServo(SHOULDER_SERVO, angles.theta1);
moveServo(ELBOW_SERVO, angles.theta2);
moveServo(WRIST_SERVO, angles.theta3);
```

## References

- See [Kinematics.md](docs/Kinematics.md) for detailed mathematical derivations
- See [ik_proof.JPG](docs/ik_proof.JPG) for inverse kinematics proof diagrams

## Author

**Mohammed Adeel Ahmed** - [@mohdadeelahmed](https://github.com/mohdadeelahmed)

