cmake_minimum_required(VERSION 3.5)
project(ros_trees_examples)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

## Generate actions, messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ActuateGripper.action"
  "action/MoveToNamedPose.action"
  "action/MoveToPose.action"
  "msg/Object.msg"
  "srv/FindObjects.srv"
  "srv/GetSyncedImages.srv"
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  nodes/bottle_binner_backend.py
  nodes/bottle_binner_tree.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
