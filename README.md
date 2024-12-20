#This is the ReadMe for the assignment of Drobot Inc

## Prerequisites

Before using ensure you have the following dependencies installed:

- ROS2 `Humble` distribution
- Additional packages:
  - `gazebo*`
  - `xacro`
  - `rmw_cyclonedds_cpp`
  - `nav*`
  - `AprilTag ROS 2 Package*`
  - `cv_bridge*`

## Usage

  - Source your build workspace every time before running any files or nodes:

    ```bash
    source <path_to_your_workspace>/install/setup.bash
    ```

  - To launch the Gazebo simulation and spawn the robot with all plugins:

    ```bash
    ros2 launch cube_bringup gazebo_bringup.launch.py
    ```
    
  - For launching Apriltag Detection and Docking:

    ```bash
    ros2 launch cube_navigation apriltag_navigation.launch.py
    ```

