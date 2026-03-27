cmake_minimum_required(VERSION 3.0.2)
project(airsim_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  VelCmd.msg
)

add_service_files(
  FILES
  Land.srv
  Reset.srv
  Takeoff.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)
