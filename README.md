# Inverse-Dynamics-Control-for-Scara-Manipulator
This project involves trajectory generation in the operational space of the Scara manipulator. The generated trajectory follows the trapezoidal velocity profile. The generated trajectory is then fed to second order inverse differential kinematics algorithm to create reference joint values which are then used in the inverse dynamics control.

## Trajectory Generation

The generated trajectory follows the trapazoidal velocity profile as shown below. 

![image](https://user-images.githubusercontent.com/69100847/209340568-b82fa752-71ca-4d13-b6a7-d6b57360d58f.png)
