# local_mapping_ros

Local mapper package. The role of the local mapper is to take the RGB-D camera images and output the cone classes and positions in 3D space.

Configurations such as tf names or topics can be updated in the launch file of this package.

## Inputs

-  `/camera/color/image_raw` (`sensor_msgs/Image`): Color image
- `/camera/depth/image_rect_raw` (`sensor_msgs/Image`): Depth image
- `/camera/depth/camera_info` (`sensor_msgs/CameraInfo`): Depth camera intrinsic parameters

## Outputs

- `/cones` (`sensor_msgs/ConeArray`): Cone array in car axis
- `/local_cone_markers` (`visualization_msgs/MarkerArray`): Array of markers for visualization in RViz or Foxglove

## Requirements
- ROS 2 Humble
- CMake
- LibTorch
- [lart_msgs](https://github.com/FSLART/lart_msgs)
- C++17 compiler

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
- Run the command `docker run --gpus all local_mapping`.
