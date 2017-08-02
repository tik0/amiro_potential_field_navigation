cmake_minimum_required(VERSION 2.8.3)
project(potential_field_navigation)

find_package(OpenMP REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        amiro_description
        amiro_gazebo
        amiro_msgs
        grid_map_core
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

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(image_to_potentialfield_node
        src/image_to_potentialfield_node.cpp)

target_link_libraries(image_to_potentialfield_node
        ${catkin_LIBRARIES}
        )

file(COPY "patter" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

add_compile_options(-std=c++11)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-deprecated-declarations -O3 -Wno-write-strings -std=c++11")