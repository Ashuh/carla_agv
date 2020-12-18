cmake_minimum_required(VERSION 2.8.3)
project(carla_vehicle_pose_publisher)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(carla_vehicle_pose_publisher_node src/carla_vehicle_pose_publisher.cpp)
target_link_libraries(carla_vehicle_pose_publisher_node ${catkin_LIBRARIES})
add_dependencies(carla_vehicle_pose_publisher_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})