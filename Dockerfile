FROM fslart/torch-opencv-ros

ENV DEBIAN_FRONTEND=noninteractive

# install dependencies
RUN apt update
RUN apt upgrade -y
RUN apt install git build-essential libeigen3-dev gdb -y

RUN apt install python3-rosdep python3-colcon-common-extensions ros-humble-ament-cmake ros-humble-ament-package -y

# add lart_msgs
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src
RUN git clone -b dev https://github.com/FSLART/lart_msgs.git
WORKDIR /ros2_ws

RUN apt install ros-humble-foxglove-bridge -y

# init rosdep
RUN rosdep init
RUN rosdep update

# copy the ros package and build it
RUN mkdir -p /ros2_ws/src/local_mapping_ros
COPY . /ros2_ws/src/local_mapping_ros
WORKDIR /ros2_ws

# install dependencies
RUN rosdep install -i --from-path src --rosdistro humble -y

# build the messages
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
 cd /ros2_ws && \
 colcon build --packages-select lart_msgs"

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