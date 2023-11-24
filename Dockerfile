FROM fslart/torch-opencv-ros

ENV DEBIAN_FRONTEND=noninteractive

# install dependencies
RUN apt update
RUN apt install git build-essential libeigen3-dev libcgal-dev -y

# clone, build and install the core library
WORKDIR /temp
RUN git clone -b dev https://github.com/FSLART/local_mapping_core.git
WORKDIR /temp/local_mapping_core
# build the library
RUN mkdir build
WORKDIR /temp/local_mapping_core/build
RUN cmake ..
RUN make -j8
# install
RUN make install
WORKDIR /home/fslart
RUN rm -rf /temp

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

# build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
 cd /ros2_ws && \
 colcon build"