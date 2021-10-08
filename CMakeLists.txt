cmake_minimum_required(VERSION 3.0.2)
project(ros_trees_examples)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  )

add_message_files(
  FILES
  Object.msg
  )

add_service_files(
  FILES
  FindObjects.srv
  GetSyncedImages.srv
  )

add_action_files(
  FILES
  ActuateGripper.action
  MoveToNamedPose.action
  MoveToPose.action
  )

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime
  )

include_directories(
  # include
  # ${catkin_INCLUDE_DIRS}
  )
