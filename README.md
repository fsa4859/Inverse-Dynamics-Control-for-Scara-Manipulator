# Inverse-Dynamics-Control-for-Scara-Manipulator
This project involves trajectory generation in the operational space of the Scara manipulator. The generated trajectory follows the trapezoidal velocity profile. The generated trajectory is then fed to second order inverse differential kinematics algorithm to create reference joint values which are then used in the inverse dynamics control.

## Trajectory Generation

The generated trajectory follows the trapazoidal velocity profile as shown below. 

![image](https://user-images.githubusercontent.com/69100847/209340568-b82fa752-71ca-4d13-b6a7-d6b57360d58f.png)

The required trajectory has to pass through the following points

**point 1**: (0,-0.8,0) at t=0 seconds

**point 2**: (0,-0.8,0.5) at t=0.6 seconds

**point 3**: (0.5,-0.6,0.5) at t=2 seconds

**point 4**: (0.8,0,0.5) at t=3.4 seconds

**point 5**: (0.8,0,0) at t=4 seconds

Anticipation of 0.2 seconds was applied to ensure smooth trajectory.

## Results

### Position Trajectory

![image](https://user-images.githubusercontent.com/69100847/209341657-acbb05f6-45c6-49c6-b27f-3e04911d1e7e.png)

### Velocity

![image](https://user-images.githubusercontent.com/69100847/209341728-99b718ef-690a-44ad-ae35-16c2dfef9179.png)

### Acceleration

![image](https://user-images.githubusercontent.com/69100847/209341768-bbe281f2-cb9c-45b8-b087-4655c3d77955.png)


## Part 2

The second part of the project consists of three main steps 

* Feeding the generated trajectory to a second order inverse differential kinematics.
* Derive the dynamics of SCARA manipulator.
* Implement the inverse dynamics control scheme.


