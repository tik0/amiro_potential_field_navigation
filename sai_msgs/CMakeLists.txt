cmake_minimum_required(VERSION 2.8.3)
project(sai_msgs)


find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs geometry_msgs nav_msgs message_generation sensor_msgs
)

add_message_files(
  FILES
  Int32MultiArrayStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

