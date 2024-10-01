# NavRover - ROS2 Jazzy Branch

## Overview

**NavRover** is a ROS-based platform designed for advanced 2D/3D mapping and autonomous navigation. This repository contains the ROS2 Jazzy branch of the project, which includes enhancements and features developed for the ROS2 environment.

## Features

- **4-Wheeled Rocker Mechanism**: Provides stability during navigation.
- **Gazebo Simulation Environment**: Enables testing and visualization of the rover's capabilities.
- **Teleoperation**: Remote control functionality integrated for testing and operation.

## TODO

- **Implement 2D Mapping**
- **Implement 3D Mapping**
- **Implement Autonomous Navigation**

## Getting Started

### Prerequisites

- **Operating System**: Ubuntu 24.04 
- **ROS2**: Installed version `Jazzy`
- **Gazebo**: Harmonic or compatible version


### Installation

1. Clone the  `ros2-jazzy` repository:

   ```bash
   git clone --single-branch --branch ros2-jazzy https://github.com/jj7258/NavRover.git   ```

2. Build the package:

   ```bash
   colcon build
   ```

3. Source the workspace:

   ```bash
   source install/setup.bash
   ```

## Usage

1. Launch the simulation:

   ```bash
   ros2 launch navrover_gazebo gazebo.launch.py
   ```

2. Use teleoperation commands to control the rover:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```


## Acknowledgments

- Special thanks to **Jerin Peter** for invaluable guidance and support throughout the project.
- Thanks to the **ROS Community** for their resources and support that have been instrumental in the development of NavRover.
- Inspired by the buildspace community and mentors.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

For any inquiries or feedback, please reach out via [GitHub Issues](https://github.com/jj7258/NavRover/issues) or contact me directly.

---
Stay tuned for updates as I continue to enhance the NavRover project!
---
