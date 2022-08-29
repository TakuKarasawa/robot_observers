cmake_minimum_required(VERSION 3.0.2)
project(robot_observers)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    multi_robot_msgs
    object_color_detector_msgs
    object_detector_msgs
    roscpp
    rospy
    tf2_geometry_msgs
    tf2_ros
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(robot_observers_node
    src/robot_observers.cpp
    src/robot_observers_node.cpp
)
target_link_libraries(robot_observers_node ${catkin_LIBRARIES})
