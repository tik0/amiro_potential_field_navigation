cmake_minimum_required(VERSION 2.8.3)
project(rsb_to_ros_bridge)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions("-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sai_msgs
  sensor_msgs
  cv_bridge
  image_transport
  compressed_image_transport
  rosgraph_msgs
)

find_package(PkgConfig)
find_package(RSB REQUIRED)
find_package(RSC REQUIRED)
find_package(RST REQUIRED COMPONENTS sandbox)
find_package(OpenCV 3.0 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES rsb_to_ros_bridge
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${RSB_INCLUDE_DIRS}
  ${RST_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  "$ENV{MUROX_PROJECT}/includes"

  include
)

LINK_DIRECTORIES (
  ${RSB_LIBRARY_DIRS}
  ${RSC_LIBRARY_DIRS}
  ${OPENCV_LIBRARY_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


add_executable(rst_vision_image_to_ros_sensormsgs_image
  src/rst_vision_image_to_ros_sensormsgs_image.cpp
)

target_link_libraries(rst_vision_image_to_ros_sensormsgs_image
  ${catkin_LIBRARIES}
  ${RSB_LIBRARIES}
  ${RST_LIBRARIES}
  ${RSTSANDBOX_LIBRARIES}
  ${OPENCV_LIBRARIES}
)

add_executable(rst_vision_laserscan_to_ros_sensormsgs_Laserscan
  src/rst_vision_laserscan_to_ros_sensormsgs_Laserscan.cpp
)

target_link_libraries(rst_vision_laserscan_to_ros_sensormsgs_Laserscan
  ${catkin_LIBRARIES}
  ${RSB_LIBRARIES}
  ${RST_LIBRARIES}
  ${RSTSANDBOX_LIBRARIES}
  ${OPENCV_LIBRARIES}
)

add_executable(rst_value_array_to_ros_int_array
  src/rst_value_array_to_ros_int_array.cpp
)

target_link_libraries(rst_value_array_to_ros_int_array
  ${catkin_LIBRARIES}
  ${RSB_LIBRARIES}
  ${RST_LIBRARIES}
  ${RSTSANDBOX_LIBRARIES}
)

add_executable(rst_pose_to_ros_posestamped
  src/rst_pose_to_ros_posestamped.cpp
)

target_link_libraries(rst_pose_to_ros_posestamped
  ${catkin_LIBRARIES}
  ${RSB_LIBRARIES}
  ${RST_LIBRARIES}
  ${RSTSANDBOX_LIBRARIES}
)

add_executable(rst_pose_to_ros_navmsgs_odometry
  src/rst_pose_to_ros_navmsgs_odometry.cpp
)

target_link_libraries(rst_pose_to_ros_navmsgs_odometry
  ${catkin_LIBRARIES}
  ${RSB_LIBRARIES}
  ${RST_LIBRARIES}
  ${RSTSANDBOX_LIBRARIES}
)

add_executable(rsb_twb_to_ros_navmsgs_odometry
  src/rsb_twb_to_ros_navmsgs_odometry.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rsb_to_ros_bridge/types/loc.pb.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rsb_to_ros_bridge/types/enum.pb.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rsb_to_ros_bridge/types/shapes.pb.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rsb_to_ros_bridge/types/pose.pb.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rsb_to_ros_bridge/types/vertex.pb.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rsb_to_ros_bridge/types/rotation.pb.cc
)

target_link_libraries(rsb_twb_to_ros_navmsgs_odometry
  ${catkin_LIBRARIES}
  ${RSB_LIBRARIES}
  ${RST_LIBRARIES}
  ${RSTSANDBOX_LIBRARIES}
)

add_executable(event_to_ros_clock src/event_to_ros_clock.cpp)

target_link_libraries(event_to_ros_clock
    ${catkin_LIBRARIES}
    ${RSB_LIBRARIES}
    ${RST_LIBRARIES}
)

ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

add_dependencies(rst_vision_image_to_ros_sensormsgs_image sai_msgs_gencpp)
add_dependencies(rst_value_array_to_ros_int_array sai_msgs_gencpp)
