cmake_minimum_required(VERSION 3.8)
project(local_mapping_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(TORCH_PATH /usr/local/lib/libtorch)
# set(CMAKE_PREFIX_PATH /usr/local/lib/libtorch)
set(CMAKE_CUDA_ARCHITECTURES 75)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(lart_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Torch REQUIRED PATHS ${TORCH_PATH})
find_package(CUDA REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

include_directories(include ${OpenCV_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${TORCH_INCLUDE_DIRS})

# add or remove this to enable or disable CUDA
# add_compile_definitions(WITH_CUDA)

add_executable(local_mapper_node 
  src/local_mapper_node.cpp
  src/LocalMapper.cpp 
  src/Utils.cpp
  src/cnn/ConeDetector.cpp 
  src/cnn/DAMO.cpp
  src/vision/ReconstructionFromDepth.cpp
  src/vision/RGBCamera.cpp
  src/vision/RGBDCamera.cpp
  src/vision/Utils.cpp
)
ament_target_dependencies(local_mapper_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs lart_msgs tf2 tf2_ros tf2_eigen cv_bridge OpenCV Torch)

target_link_libraries(local_mapper_node ${OpenCV_LIBRARIES} ${Eigen3_LIBS} ${TORCH_LIBRARIES} ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY} ${CUDA_cusolver_LIBRARY} ${CUDA_cusparse_LIBRARY})

# Install launch files
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

# install the model
install(DIRECTORY 
  model
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  local_mapper_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
