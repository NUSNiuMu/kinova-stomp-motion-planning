# Kinova STOMP Motion Planning Project

## 📋 Project Overview

This project implements a robot manipulator collision avoidance path planning system based on the **STOMP (Stochastic Trajectory Optimization for Motion Planning)** algorithm. Developed in MATLAB, the project supports multiple robot platforms and provides a complete solution from basic path planning to complex constraint scenarios.

**Course Information:**
- Course: EE5112 Human Robot Interaction
- Project: Project 2 - Robot Manipulator Collision Avoidance Path Planning using STOMP
- Academic Year: AY 2025/2026
- Instructor: Dr. Lin Zhao (School of ECE, NUS)

## 🎯 Project Objectives

This project consists of 5 progressive tasks that gradually implement and improve the application of the STOMP algorithm in manipulator motion planning:

1. **Task 1**: Complete the example code to implement basic collision avoidance path planning for Kinova Gen3 manipulator
2. **Task 2**: Extend support for multiple robot platforms (ABB IRB120, Franka Panda, Kuka IIWA, UR series, etc.)
3. **Task 3**: Replace MATLAB built-in functions with forward kinematics based on Product of Exponentials (PoE) formula
4. **Task 4**: Design custom obstacle avoidance scenarios for path planning in complex obstacle environments
5. **Task 5**: Add end-effector orientation constraints to achieve direction-aligned motion planning

## 📁 Project Structure

```
kinova-stomp-motion-planning/
├── Task1/                    # Task 1: Basic STOMP Implementation
│   ├── KINOVA_STOMP_Path_Planning.mlx
│   ├── helperSTOMP.m
│   ├── updateJointsWorldPosition.m
│   ├── stompDTheta.m
│   ├── stompSamples.m
│   ├── stompObstacleCost.m
│   └── ...
├── Task2/                    # Task 2: Multi-Robot Platform Support
│   └── ...
├── Task3/                    # Task 3: PoE Forward Kinematics Implementation
│   └── ...
├── Task4/                    # Task 4: Custom Obstacle Avoidance Scenarios
│   ├── RunTask4.m
│   ├── helperCreateObstaclesTask4.m
│   ├── helperVoxelizeOrientedBox.m
│   └── ...
├── Task5/                    # Task 5: End-Effector Orientation Constraints
│   ├── RunTask5_Compare.m
│   ├── README.md
│   └── ...
├── Report/                   # Project Report
│   ├── Report.tex
│   ├── Report.pdf
│   └── Figures/
├── EE5112_Project_2_Collision-Avoidance-AY25-26-Sem1 (1).pdf
├── 项目要求.md
└── README.md
```

## ⚙️ Requirements

### MATLAB Version
- MATLAB R2020b or higher

### Required Toolboxes
- **Robotics System Toolbox** - For robot model loading and kinematics computation
- **Statistics and Machine Learning Toolbox** - For multivariate Gaussian sampling (`mvnrnd` function)

### Installation Instructions
1. Open MATLAB
2. Go to `Add-Ons` → `Get Add-Ons`
3. Search and install the following toolboxes:
   - Robotics System Toolbox
   - Statistics and Machine Learning Toolbox

## 🚀 Quick Start

### Task 1: Basic Path Planning

1. Open MATLAB and set the current directory to the project root
2. Navigate to the `Task1` folder
3. Run the Live Script:
   ```matlab
   open('KINOVA_STOMP_Path_Planning.mlx')
   ```
4. Click the `Run` button to execute the complete script

### Task 2: Multi-Robot Platforms

1. Navigate to the `Task2` folder
2. Run the main script:
   ```matlab
   RunLiveScript
   ```
3. The script will automatically load the specified robot model and execute path planning

### Task 3: PoE Forward Kinematics

1. Navigate to the `Task3` folder
2. Run the main script:
   ```matlab
   RunLiveScript
   ```
3. The system will use the PoE formula to replace `getTransform()` for forward kinematics computation

### Task 4: Custom Obstacle Avoidance Scenarios

1. Navigate to the `Task4` folder
2. Run the main script:
   ```matlab
   RunTask4
   ```
3. Modify the `scenario_id` parameter in the script to select different scenarios (1-4)

### Task 5: Orientation Constraint Planning

1. Navigate to the `Task5` folder
2. View detailed instructions:
   ```matlab
   open('README.md')
   ```
3. Run the comparison script:
   ```matlab
   RunTask5_Compare
   ```
4. The script will compare path planning results with and without orientation constraints

## 📝 Core Algorithms and Implementation

### STOMP Algorithm Flow

1. **Initialize Trajectory**: Generate initial trajectory using linear interpolation or smooth initialization
2. **Sampling Phase**: Generate multiple noisy sample trajectories for each time step
3. **Cost Evaluation**: Compute total cost for each sample trajectory (obstacle cost + smoothness cost)
4. **Weighted Update**: Weight sample trajectories based on cost and update current trajectory
5. **Iterative Optimization**: Repeat steps 2-4 until convergence or maximum iterations reached

### Key Functions

- `helperSTOMP.m`: STOMP main loop, controls iterative optimization process
- `stompSamples.m`: Generates multivariate Gaussian sample trajectories
- `stompTrajCost.m`: Computes total trajectory cost (obstacle + smoothness + orientation constraints)
- `stompObstacleCost.m`: Computes obstacle cost based on Signed Distance Field (SDF)
- `stompDTheta.m`: Computes gradient estimate (weighted noise summation)
- `updateJointsWorldPosition.m`: Forward kinematics computation (implemented using PoE in Task 3)

### PoE Forward Kinematics

Task 3 implements forward kinematics based on the Product of Exponentials formula:

\[
T(\theta) = e^{[\xi_1]\theta_1} e^{[\xi_2]\theta_2} \cdots e^{[\xi_n]\theta_n} M(0)
\]

Where:
- $\xi_i$ is the screw axis of the $i$-th joint
- $\theta_i$ is the angle of the $i$-th joint
- $M(0)$ is the homogeneous transformation matrix at initial configuration

## 🎨 Visualization Features

The project provides rich visualization capabilities:

- **3D Robot Model Display**: Real-time display of robot configuration and trajectory
- **Obstacle Visualization**: 3D display of voxelized obstacles
- **Trajectory Comparison**: Overlay display of multiple trajectories for comparative analysis
- **End-Effector Path**: Highlighted display of end-effector motion path
- **Orientation Constraint Visualization**: Display of end-effector axis direction and alignment target

## 📊 Experimental Results

The project has been validated on multiple robot platforms:

- **Kinova Gen3**: 7-DOF manipulator, primary test platform
- **ABB IRB120**: 6-DOF industrial robot
- **Franka Emika Panda**: 7-DOF collaborative robot
- **Kuka IIWA 7**: 7-DOF lightweight robot
- **Universal Robots (UR3/UR5/UR10)**: 6-DOF collaborative robots
- **Rethink Sawyer**: 7-DOF single-arm robot

All platforms achieved 100% path planning success rate.

## 🔧 Parameter Configuration

### STOMP Core Parameters

- `stomp_nDiscretize`: Number of trajectory discretization points (Recommended: 20-30)
- `stomp_nPaths`: Number of sample paths (Recommended: 20-30)
- `stomp_nIter`: Maximum number of iterations (Recommended: 50-100)
- `stomp_eta`: Step size coefficient (Recommended: 0.1-0.3)

### Obstacle Parameters

- `voxel_size`: Voxel size (Recommended: 0.02-0.05 m)
- `obstacle_margin`: Obstacle safety margin (Recommended: 0.05-0.1 m)

### Orientation Constraint Parameters (Task 5)

- `task5_keep_axis`: End-effector axis to align ('x'|'y'|'z')
- `task5_world_axis`: World coordinate system target axis vector
- `task5_penalty_weight`: Orientation penalty weight
- `task5_angle_threshold_deg`: Angle threshold (degrees)

## 📚 References

1. Kalakrishnan, M., et al. "STOMP: Stochastic trajectory optimization for motion planning." *2011 IEEE international conference on robotics and automation*. IEEE, 2011.

2. Lynch, K. M., & Park, F. C. *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press, 2017.

3. MATLAB Robotics System Toolbox Documentation: https://www.mathworks.com/help/robotics/

## 👥 Team Members

- Wu Zining
- Niu Mu
- Zhao Jinqiu

## 📄 License

This project is a course assignment project, for learning and research purposes only.

## 📧 Contact

For questions or suggestions, please contact:
- Instructor: Dr. Lin Zhao (elezhli@nus.edu.sg)
- School: School of Electrical and Computer Engineering, National University of Singapore

## 🙏 Acknowledgments

Thanks to the EE5112 course team for providing example code and technical support.

---

**Last Updated**: November 2025
