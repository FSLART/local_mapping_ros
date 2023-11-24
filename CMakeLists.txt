cmake_minimum_required(VERSION 3.8)
project(local_mapping_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(local_mapping_core REQUIRED)

include_directories(include ${local_mapping_core_INCLUDE_DIRS})

add_executable(local_mapping_ros_node src/LocalMapper.cpp)
ament_target_dependencies(local_mapping_ros_node rclcpp sensor_msgs local_mapping_core)

install(TARGETS
  local_mapping_ros_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
