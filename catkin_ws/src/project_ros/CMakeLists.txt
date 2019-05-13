cmake_minimum_required(VERSION 2.8.3)
project(project_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)



