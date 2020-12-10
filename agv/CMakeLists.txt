cmake_minimum_required(VERSION 2.8.3)
project(agv)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
