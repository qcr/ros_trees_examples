cmake_minimum_required(VERSION 3.0.2)
project(ros_trees_examples)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

add_action_files(
  FILES
  ActuateGripper.action
  )

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime
  )

include_directories(
  # include
  # ${catkin_INCLUDE_DIRS}
  )
