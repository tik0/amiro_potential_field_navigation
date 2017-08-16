cmake_minimum_required(VERSION 2.8.3)
project(potential_field_navigation)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

find_package(OpenMP REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  amiro_description
  amiro_gazebo
  amiro_msgs
  grid_map_core
  grid_map_msgs
  grid_map_cv
  grid_map_ros
  nav_msgs
  roscpp
  rospy
  rviz_textured_quads
  sensor_mockup
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  )

catkin_package(
)


if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-deprecated-declarations -O3 -Wno-write-strings -std=c++11")

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/
)
#image_to_potentialfield_node
add_executable(image_to_potentialfield_node
  src/image_to_potentialfield_node.cpp)

target_link_libraries(image_to_potentialfield_node
  ${catkin_LIBRARIES}
  )

#potentialfield_to_gridmap_node
add_executable(potentialfield_to_gridmap_node
  src/potentialfield_to_gridmap_node.cpp)

target_link_libraries(potentialfield_to_gridmap_node
  ${catkin_LIBRARIES}
  )

#image_selecter_node
add_executable(image_selecter_node
  src/image_selecter_node.cpp)

target_link_libraries(image_selecter_node
  ${catkin_LIBRARIES}
  Qt5::Widgets
  )

#amiro_odom_to_potentialfield_node
add_executable(amiro_odom_to_potentialfield_node
  src/amiro_odom_to_potentialfield_node.cpp)

target_link_libraries(amiro_odom_to_potentialfield_node
  ${catkin_LIBRARIES}
  )

#potentialfield_merger_node
add_executable(potentialfield_merger_node
  src/potentialfield_merger_node.cpp)

target_link_libraries(potentialfield_merger_node
  ${catkin_LIBRARIES}
  )

#potentialfield_to_amiro_kinematic_node
add_executable(potentialfield_to_amiro_kinematic_node
  src/potentialfield_to_amiro_kinematic_node.cpp)

target_link_libraries(potentialfield_to_amiro_kinematic_node
  ${catkin_LIBRARIES}
  )

file(COPY "patter" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

