# legged_control_cpp

[![ci](https://github.com/ermolenkodev/legged_control_cpp/actions/workflows/ci.yml/badge.svg)](https://github.com/ermolenkodev/legged_control_cpp/actions/workflows/ci.yml)
[![CodeQL](https://github.com/ermolenkodev/legged_control_cpp/actions/workflows/codeql-analysis.yml/badge.svg)](https://github.com/ermolenkodev/legged_control_cpp/actions/workflows/codeql-analysis.yml)

## About legged_control_cpp
The project is a ground-up implementation of Rigid Body Dynamics and high-level control algorithms for robotics manipulators and legged robots.
The primary goal is to develop all these algorithms from scratch (possibly excluding the QP solver), and to do so without utilizing any external robotics libraries, such as Pinocchio, Drake, etc.

## Planned features
- [x] Model of kinematic tree
- [x] Basic URDF parsing
- [x] Simple visualization
- [ ] Rigid Body Dynamics
  - [x] Recursive Newton-Euler Algorithm
  - [x] Composite Rigid Body Algorithm
  - [x] Jacobian computation
  - [ ] Classic acceleration computation
- [ ] Manipulator Control
  - [ ] Joint Space Control
    - [x] Simple PD Control
    - [ ] PD Control with Gravity Compensation
    - [ ] Feedforward Control
    - [ ] Inverse Dynamics Control
  - [ ] Operational Space Control
  - [ ] Contact consistent dynamics
- [ ] Legged Robot Control
  - [ ] Simulation of legged robot
  - [ ] ...

## Usage
The Rigid Body Dynamics implementation is encapsulated in the [legged_control_lib](src/legged_control_cpp) library. This library is linked to various executable examples located in the samples directory.
> **Note:**
> And executables should be run from the project's root directory.
> By default executables are located in the corresponding build directory.
### Samples
#### URDF parsing sample
`urdf_sample`: A simple example demonstrating how to parse a URDF file.
#### Joint Space contol sample
`joint_space_control`: This is an implementation of joint space control tailored for a UR-5 manipulator. It generates an output file named `joint_space_control.log` in the project's root directory. This file contains the simulated joint trajectory and can be visualized using the [rviz trajectory visualizer](https://github.com/ermolenkodev/rviz_trajectory_player)


## Visualization
The project uses [rviz trajectory visualizer](https://github.com/ermolenkodev/rviz_trajectory_player) for visualization.
The visualizer is a ROS package that plays back a trajectory of a robot in rviz.
For installation instructions, please refer to the projects' [repository](https://github.com/ermolenkodev/rviz_trajectory_player).

To replay the trajectory, run the following command:
```bash
 ros2 launch rviz_trajectory_player rviz_trajectory_player.launch.py robot_file:=<absolute_path_to_urdf|xacro_file> trajectory_file:=<absolute_path_to_trajectory_file>
```
## More Details
> This project structure was derived from the [cpp_starter_project](https://github.com/cpp-best-practices/cmake_template). As a result, you may encounter instances of the template project's name in the CMake files or within the documentation.
Please also be aware that while instructions for Windows and macOS setup are provided, they may not be fully complete or thoroughly tested. This project has been primarily tested on Ubuntu. However, it's anticipated to function on Windows and macOS systems, provided that the necessary dependencies have been properly installed.
 * [Dependency Setup](README_dependencies.md)
 * [Building Details](README_building.md)
 * [Docker](README_docker.md)

### Note about the License
This project closely follows the API and notation used in spatial v2, and as a result,
it is considered a derivative work of this library and is licensed with GPL v3.
