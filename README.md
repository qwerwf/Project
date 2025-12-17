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
Clone the repository:textgit clone https://github.com/yourusername/robot-project.git
cd robot-project
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
