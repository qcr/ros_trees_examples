cmake_minimum_required(VERSION 3.0.2)
project(ros_trees_examples)

find_package(catkin REQUIRED)

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

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES ros_trees_examples
  #  CATKIN_DEPENDS other_catkin_pkg
  #  DEPENDS system_lib
  )

include_directories(
  # include
  # ${catkin_INCLUDE_DIRS}
  )
