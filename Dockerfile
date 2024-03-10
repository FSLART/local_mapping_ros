FROM fslart/onnx-opencv-ros

ENV DEBIAN_FRONTEND=noninteractive

# install dependencies
RUN apt update
RUN apt install git build-essential libeigen3-dev -y

RUN apt install python3-rosdep python3-colcon-common-extensions -y

# add lart_msgs
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src
RUN git clone -b dev https://github.com/FSLART/lart_msgs.git
WORKDIR /ros2_ws

# build lart_msgs
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
 cd /ros2_ws && \
 colcon build --packages-select lart_msgs"

# install foxglove bridge
RUN apt install ros-humble-foxglove-bridge -y

# install ros dependencies
RUN apt install -y ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-visualization-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2 \
    ros-humble-tf2-eigen \
    ros-humble-cv-bridge

# copy the ros package and build it
RUN mkdir -p /ros2_ws/src/local_mapping_ros
COPY . /ros2_ws/src/local_mapping_ros
WORKDIR /ros2_ws

# build this package
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
 cd /ros2_ws && \
 source install/setup.bash && \
 colcon build --parallel-workers 6 --symlink-install --packages-select local_mapping_ros --cmake-args -DCMAKE_BUILD_TYPE=Release"

# copy the torchscript models
COPY model/ /ros2_ws/model/

CMD /bin/bash -c "source /opt/ros/humble/setup.bash && \
 cd /ros2_ws && \
 source install/local_setup.bash && \
 ros2 launch local_mapping_ros mapper.launch"

