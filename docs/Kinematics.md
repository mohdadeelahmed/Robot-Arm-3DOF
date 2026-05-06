# Inverse Kinematics for 2-Link Robotic Arm

## System Constants
The arm consists of two primary segments. The lengths are measured from their exact axes of rotation to ensure mathematical accuracy, with $L_2$ accounting for the physical offset of the claw.

* **$L_1$ (Shoulder to Elbow):** 97.4 mm
* **$L_2$ (Elbow to End Effector):** 222.9 mm

## 1. Safety and Reachability Check
Before calculating joint angles, the system must verify if the target coordinate $(X, Y)$ is physically reachable. Attempting to reach an impossible coordinate will result in a fatal math error (calculating the square root of a negative number) and potential hardware damage.

The straight-line distance to the target is defined as:
$$ D = \sqrt{X^2 + Y^2} $$

The target is valid only if it falls within the physical limits of the arms:
$$ |L_1 - L_2| \le D \le (L_1 + L_2) $$
$$ 125.5 \text{ mm} \le D \le 320.3 \text{ mm} $$

If $D$ falls outside this range, the firmware aborts the movement command.

## 2. Elbow Angle ($\theta_2$) Calculation
We use the Law of Cosines to determine the elbow angle required to reach the target $(X, Y)$.

$$ \cos(\theta_2) = \frac{X^2 + Y^2 - L_1^2 - L_2^2}{2 L_1 L_2} $$

Substituting the system constants:
$$ \cos(\theta_2) = \frac{X^2 + Y^2 - (97.4)^2 - (222.9)^2}{2(97.4)(222.9)} $$
$$ \cos(\theta_2) = \frac{X^2 + Y^2 - 59171.17}{43421.16} $$

To find the final angle in radians:
$$ \theta_2 = \arccos \left( \frac{X^2 + Y^2 - 59171.17}{43421.16} \right) $$

## 3. Shoulder Angle ($\theta_1$) Calculation
With the elbow angle solved, we calculate the shoulder angle by combining the angle to the target coordinate with the inner geometry of the arm linkage.

$$ \theta_1 = \text{atan2}(Y, X) - \text{atan2}(L_2 \sin(\theta_2), L_1 + L_2 \cos(\theta_2)) $$

Substituting the system constants:
$$ \theta_1 = \text{atan2}(Y, X) - \text{atan2}(222.9 \sin(\theta_2), 97.4 + 222.9 \cos(\theta_2)) $$

*Note: The firmware converts the final $\theta_1$ and $\theta_2$ radian values into degrees before transmitting the PWM signals to the hardware.*
