# Franka Pybullet ROS

This project provides a PyBullet-based simulation environment for the Franka Emika Panda robot, integrated with ROS functionality. It allows users to simulate the Panda robot in both physics and visual modes, offering various control and interaction features.

## Core Concept: Teleoperation Assistance

The core of this project lies in its teleoperation assistance functionality. We utilize GraspNet for grasp pose estimation, generating potential grasp poses. These poses are then incorporated into a 6D Gaussian field, which generates grasp guidance for every point in space. This guidance is superimposed on the Quest-controlled Franka end-effector, providing assisted robot grasping.

Experimental results demonstrate that this assistance feature significantly improves grasp success rates and reduces the cognitive load on operators.

Key benefits:
1. Enhanced grasp success rates
2. Reduced operator mental workload
3. Intuitive and seamless integration of AI-assisted grasping with human control

This innovative approach bridges the gap between autonomous robotic grasping and human-controlled teleoperation, offering a more efficient and user-friendly solution for remote manipulation tasks.


## Main Features

1. PyBullet-based Franka Panda robot simulation
2. ROS integration with support for publishing and subscribing to relevant topics
3. Both physics and visual simulation modes
4. Support for position, velocity, and torque control
5. Gripper control functionality
6. Camera image acquisition and publishing
7. Object manipulation and grasping capabilities
8. VR control interface

## Main Components

### FrankaPandaEnv
Base simulation environment class, defined in `env.py`. It initializes the PyBullet environment, loads the Panda robot model, and provides basic simulation stepping methods.

### FrankaPandaEnvRosPhysics
Physics simulation environment class, defined in `env_ros.py`. It inherits from FrankaPandaEnv and adds ROS-related functionality, such as publishing joint states and subscribing to control commands.

### FrankaPandaEnvRosVisual
Visual simulation environment class, also defined in `env_ros.py`. It provides functionality for capturing camera images and publishing image data.

### FrankaPanda
Robot control class, defined in `panda_robot.py`. It encapsulates low-level control methods for the robot, such as setting joint positions, velocities, and torques.

### GraspNode
Grasping node class, defined in `grasp_node.py`. It implements point cloud-based grasp pose estimation functionality.

### VRControlNode
VR control node class, defined in `vr_control_node.py`. It allows control of the robot through VR devices.

## Usage

- Launch physics simulation

- Launch visual simulation  

- Run VR control

- Run grasp estimation

## Dependencies

- ROS
- PyBullet
- NumPy
- OpenCV
- roboticstoolbox
- Open3D

## Notes

- The simulation environment uses a default sampling rate of 1000Hz.
- Both complete Panda models and simplified models without hands are provided.
- Inverse kinematics calculations may be slow; optimization should be considered.
- VR control and grasp estimation features require additional hardware or sensor support.

## Contributing

Issue reports and pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License

MIT

