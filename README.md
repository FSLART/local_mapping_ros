# local_mapping_ros

Local mapper ROS wrapper.

## Requirements
- ROS 2
- CMake
- TorchScript
- C++17 compiler
- [local_mapping_core](https://github.com/FSLART/local_mapping_core)

or

- Docker


## Building

If you aren't working in this package and just want to use it, or deploying to production, Docker is recommended (takes a lot of time the first time but shouldn't break).

### Bare metal

- Install the dependencies listed above.
- Make a new directory for your workspace.
    - Example: `mkdir -p ~/catkin_ws/src`
- Put this directory in the `src` directory of the workspace.
- Run the command `source /opt/ros/humble/setup.bash`.
- From the root of the workspace run:
    - `rosdep install -i --from-path src --rosdistro humble -y`
    - `colcon build`

### Docker

- Run the command `docker build -t local_mapping .` and wait about some centuries.
