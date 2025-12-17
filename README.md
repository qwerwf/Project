# Project
 Autonomous Mobile Robots Final Project -- Two-wheel differential drive robot obstacle avoidance design
This project implements a complete navigation and control framework for a differential-drive mobile robot in a 2D environment with obstacles.
The system combines global path planning, local obstacle avoidance, path following, and robust low-level control to guide the robot safely to a target position.
1. Project Overview
The navigation pipeline is composed of four main modules:
RRT* for global path planning
Pure Pursuit for path following
Dynamic Window Approach (DWA) for local obstacle avoidance
Sliding Mode Control (SMC) for velocity tracking and torque control
Each module operates at a different level of abstraction, from high-level planning to low-level motor control.

2. System Architecture
The overall workflow is:
RRT* generates a collision-free global path as a sequence of waypoints.
Pure Pursuit converts the robot pose and waypoints into desired linear and angular velocities.
DWA is activated only when obstacles are too close and modifies the desired velocities for safe local avoidance.
Sliding Mode Control converts desired velocities into left and right wheel torques.
Robot dynamics and kinematics are simulated to update the robot state.
Summary:
RRT decides where to go
Pure Pursuit decides how to follow the path
DWA decides how to avoid obstacles
SMC decides how to apply forces to the robot

3. Modeling
3.1 Kinematics
The robot is modeled as a planar differential-drive system:
𝑥˙=𝑣cos⁡𝜃, 𝑦˙=𝑣sin⁡𝜃, 𝜃˙=𝜔
Assumptions:
Rigid body，No wheel slip，Motion on a flat 2D plane，Low-speed operation

3.2 Dynamics
The simplified dynamic model is:
𝑣˙=(𝜏_𝑟+𝜏_l)/𝑚r−𝑑_𝑣𝑣
𝜔˙=𝑏(𝜏_𝑟−𝜏_𝑙)/𝐼𝑟−𝑑_𝜔𝜔
Assumptions:
Constant mass and inertia
Linear damping
Ideal actuators
Bounded disturbances

4. Control Methodology
Sliding Mode Control (SMC)
Sliding Mode Control is used for robust velocity tracking.
Control objective: track desired linear and angular velocities
Sliding surfaces are defined using velocity errors
A smooth tanh() function is used to reduce chattering
Robust against modeling errors and disturbances

5. Motion Planning
5.1 Global Planning: RRT*
Plans a collision-free path in a known map
Works well in cluttered environments
Outputs a sequence of waypoints

5.2 Path Following: Pure Pursuit
Tracks the global path smoothly
Reduces heading errors
Prevents error accumulation during motion

5.3 Local Avoidance: DWA
Activated only when obstacles are close
Samples feasible velocity commands
Selects safe velocities based on heading, clearance, and speed

6. Simulation Environment
MATLAB-based simulation
Binary occupancy grid map
Obstacle inflation for safety margin
Real-time visualization of:
Planned path (green dashed line)
Actual robot trajectory (red line)

7. Results

The robot successfully reaches the target without collision
Actual trajectory closely follows the planned path
Velocity tracking error is close to zero
Pure Pursuit significantly improves path tracking compared to using DWA alone

8. Challenges and Solutions
Challenge:
Without Pure Pursuit, small heading errors were amplified by the dynamics, leading to large path deviations.

Solution:
Pure Pursuit was added as a path-following layer to stabilize heading errors and guide the robot along the planned trajectory.
