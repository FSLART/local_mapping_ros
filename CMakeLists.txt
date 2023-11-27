cmake_minimum_required(VERSION 3.8)
project(local_mapping_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(TORCH_PATH /usr/local/lib/libtorch)
# set(CMAKE_PREFIX_PATH /usr/local/lib/libtorch)
set(CMAKE_CUDA_ARCHITECTURES 75)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Torch REQUIRED PATHS ${TORCH_PATH})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

include_directories(include ${OpenCV_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${TORCH_INCLUDE_DIRS})

add_executable(local_mapping_ros_node 
  src/local_mapper_node.cpp
  src/LocalMapper.cpp 
  src/cnn/ConeDetector.cpp 
  src/cnn/DAMO.cpp
  src/vision/ReconstructionFromDepth.cpp
  src/vision/RGBCamera.cpp
  src/vision/RGBDCamera.cpp
  src/vision/Utils.cpp
)
ament_target_dependencies(local_mapping_ros_node rclcpp sensor_msgs geometry_msgs OpenCV Torch)

message(WARNING "TORCH_LIBRARIES ${TORCH_LIBRARIES}")

target_link_libraries(local_mapping_ros_node ${OpenCV_LIBRARIES} ${Eigen3_LIBS} ${TORCH_LIBRARIES})

install(TARGETS
  local_mapping_ros_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
