cmake_minimum_required(VERSION 3.10)
project(a1_visualizer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET (CMAKE_CXX_STANDARD 11)
SET (CMAKE_CXX_STANDARD_REQUIRED ON)
SET (CMAKE_CXX_FLAGS "-O3")
set (CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -fpermissive")


# added to supprt robot_markers, this one should be included with ros-version-orocos-kdl
find_package(orocos_kdl REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  rosbag
  geometry_msgs
  message_generation

  
  # added to support robot_markers
  kdl_parser
  tf2_kdl
  tf2_ros
  transform_graph
  urdf
  visualization_msgs


  unitree_legged_msgs
)

#  DEPENDS system_lib
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES 
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    roslib
    std_msgs 
    geometry_msgs 
    rosbag 
    message_runtime
)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${catkin_INCLUDE_DIRS}
  # added to supprt robot_markers
  ${orocos_kdl_INCLUDE_DIRS}
)


SET(A1_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/visualize.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_markers/builder.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_markers/forward_kinematics.cpp
)

add_executable(a1_visualizer
  ${A1_SOURCES} )
target_link_libraries( a1_visualizer
                          ${orocos_kdl_LIBRARIES}
                          ${catkin_LIBRARIES} 
                     )


# this is for Isaac Sim robot
SET(ISAAC_A1_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/isaac_a1_visualize.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_markers/builder.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_markers/forward_kinematics.cpp
)

add_executable(isaac_a1_visualizer
  ${ISAAC_A1_SOURCES} )
target_link_libraries( isaac_a1_visualizer
                          ${orocos_kdl_LIBRARIES}
                          ${catkin_LIBRARIES} )