# Autonomous Mobile Robot Simulation Project
##  Overview
This repository contains a MATLAB implementation of a two-wheeled differential drive mobile robot simulation, developed as a project for the "Autonomous Mobile Robots: Control and Planning" course at NYU (Fall 2025, taught by Aliasghar Moj Arab). The project covers system modeling (kinematics and dynamics), robust control using Sliding Mode Control (SMC), and motion planning with RRT* for global paths and Dynamic Window Approach (DWA) for local obstacle avoidance. It includes real-time visualization of planned vs. actual trajectories in a maze environment.

The simulation demonstrates autonomous navigation in a 10x10 m map with obstacles, ensuring safe and efficient path following.

## Features

- System Modeling: Unicycle kinematics and Lagrangian dynamics for realistic robot motion.
- Control: SMC with smooth approximation to track reference velocities robustly.
- Motion Planning: Hierarchical approach – RRT* for global planning, conditional DWA for avoidance.
- Visualization: Animated plot showing robot trajectory, planned path, and obstacle map.
- Validation: Post-simulation plots for velocity errors.

## Prerequisites

MATLAB R2023a or later.
Robotics System Toolbox (for plannerRRTStar, controllerPurePursuit, etc.).
No additional installations needed; the script is self-contained.

## Installation

```bash
Clone the repository:textgit clone https://github.com/qwerwf/Project.git
cd Project
```

Open MATLAB and run robot_project_fixed.m (the main script).

## Usage

Run the script robot_project_fixed.m in MATLAB.
The animation will display the robot navigating from [1,1,0] to [9,9,pi/2] (adjustable in code).
Customize:
Map: Edit obstacles in the map setup section.
Parameters: Tune SMC gains (lambda_v), DWA weights (alpha, beta), or Pure Pursuit lookahead.

Output: Trajectory animation and velocity error plot.

## Code Structure

Parameters: Robot physics, SMC, DWA settings.
Map Setup: Binary occupancy map with rectangular obstacles (maze-like).
Global Planning: RRT* with retry loop for reliability.
Simulation Loop: Pure Pursuit for path following, conditional DWA, SMC control, state updates, animation.
Validation: Error plots post-simulation.
Helpers: DWA function and distance calculator.

##  Standalone Test Scripts

In addition to the main navigation script, the repository includes independent test scripts for isolated validation of core components:

### kinematics_dynamics_test.m：

Run this script directly in MATLAB to compare pure kinematic (ideal instantaneous response) and full dynamic (realistic transients with mass/inertia/damping) behaviors under the same input; modify reference wheel velocities or control inputs (u_v, u_w) to generate different trajectories (e.g., circle, figure-8), and adjust robot parameters (m, I, d_v, d_w) to observe effects on acceleration, deceleration, and path realism.

### smc_test.m：

Run this script directly in MATLAB to evaluate the Sliding Mode Controller's performance on the dynamic model with desired vs. actual trajectory overlay; tune SMC parameters (lambda_v/w, eta, F, phi) to reduce tracking errors and chattering, or modify reference signals (v_d, w_d) to test robustness under different path demands (e.g., aggressive turns or varying speeds).

## Results
The robot successfully navigates mazes, with actual paths closely following planned ones. Velocity errors converge to <0.1 m/s. Example output:
Simulation Example
(Planned: green dashed; Actual: red; Robot: blue triangle)
Challenges and Solutions

Path Deviations: Initial DWA over-avoidance; fixed by tuning weights and conditional activation.
Planning Failures: RRT* probabilistic issues; addressed with retry loop and increased iterations.
Close to Obstacles: Resolved by larger map inflation and CBF-like checks.


## License
MIT License. Feel free to use and modify for educational purposes.
