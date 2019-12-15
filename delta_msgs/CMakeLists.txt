cmake_minimum_required(VERSION 2.8.3)
project(delta_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  visualization_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  Track.msg
  TrackArray.msg
  CameraTrack.msg
  CameraTrackArray.msg
  CollisionDetection.msg
  MarkerArrayStamped.msg
  VehicleGroundTruth.msg
  VehicleGroundTruthArray.msg
  LaneMarking.msg
  LaneMarkingArray.msg
  EgoStateEstimate.msg
  EgoStateEstimateArray.msg
  OncomingVehicleStateEstimate.msg
  OncomingVehicleTrajectory.msg 
  OncomingVehicleTrajectoryArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs geometry_msgs visualization_msgs message_runtime
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
