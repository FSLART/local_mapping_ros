FROM fslart/torch-opencv-ros

ENV DEBIAN_FRONTEND=noninteractive

# install dependencies
RUN apt update
RUN apt install git build-essential libeigen3-dev -y

RUN apt install python3-rosdep python3-colcon-common-extensions -y

# init rosdep
RUN rosdep init
RUN rosdep update

# copy the ros package and build it
RUN mkdir -p /ros2_ws/src/local_mapping_ros
COPY . /ros2_ws/src/local_mapping_ros
WORKDIR /ros2_ws

# install dependencies
RUN rosdep install -i --from-path src --rosdistro humble -y

RUN apt install ros-humble-foxglove-bridge -y

# build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
 cd /ros2_ws && \
 colcon build --parallel-workers 6 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

CMD /bin/bash -c "source /opt/ros/humble/setup.bash && \
 cd /ros2_ws && \
 source install/local_setup.bash && \
 cd src/local_mapping_ros/launch && \
 ros2 launch mapper.launch"