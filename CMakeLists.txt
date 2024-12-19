cmake_minimum_required(VERSION 3.0.2)
project(lab3_drones)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Add dependencies here (look for them in imported libraries in the header files)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mav_trajectory_generation
  mav_trajectory_generation_ros
)
find_package(Eigen3 REQUIRED)

message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES lab3_drawing
  LIBRARIES lab3_drawing lab3_utils
  CATKIN_DEPENDS geometry_msgs nav_msgs visualization_msgs tf2 mav_trajectory_generation mav_trajectory_generation_ros
  DEPENDS Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Add library for utils.cpp
add_library(lab3_utils
    src/drones_utils.cpp
)
target_link_libraries(lab3_utils
    ${catkin_LIBRARIES}
    Eigen3::Eigen
)

## Add library for drawing.cpp
add_library(lab3_drawing
    src/drawing.cpp
)
target_link_libraries(lab3_drawing
    lab3_utils
    ${catkin_LIBRARIES}
)

## Add drone_race executable
add_executable(drone_race
   src/drone_race.cpp
   src/drone_race_node.cpp
)

target_link_libraries(drone_race
  lab3_drawing
  lab3_utils
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

#############
## Install ##
#############

# Not for now
