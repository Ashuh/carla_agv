cmake_minimum_required(VERSION 2.8.3)
project(ackermann_drive_publisher)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  ackermann_msgs
)

generate_dynamic_reconfigure_options(
  cfg/ackermann_drive.cfg
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ackermann_drive_publisher_node src/ackermann_drive_publisher.cpp)
target_link_libraries(ackermann_drive_publisher_node ${catkin_LIBRARIES})
add_dependencies(ackermann_drive_publisher_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
