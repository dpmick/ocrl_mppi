cmake_minimum_required(VERSION 3.16)

project(mppi)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(  Eigen3 REQUIRED )


find_package(catkin REQUIRED COMPONENTS
roscpp
nav_msgs
geometry_msgs
tf
std_msgs
tf2_ros
tf2_geometry_msgs
tf2
tf2_msgs
visualization_msgs
mmpug_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES traj_lib
    CATKIN_DEPENDS 
        roscpp 
        std_msgs 
        geometry_msgs 
        nav_msgs
        tf2_ros
        tf2_msgs
        tf2_geometry_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(path
    src/path.cpp)
target_sources(path
    PUBLIC
    include/mppi/path.hpp
)

add_library(mppi
    src/mppi.cpp)
target_sources(mppi
    PUBLIC
    include/mppi/mppi.hpp
    include/mppi/path.hpp
)

add_library(ros1_utils
    src/utils_ros1.cpp)
target_sources(ros1_utils
    PUBLIC
    include/mppi/mppi.hpp
    include/mppi/path.hpp
    ${catkin_LIBRARIES}
)

add_executable(ros_node
    src/ros_node.cpp 
)
target_link_libraries(ros_node
    path 
    mppi 
    ros1_utils 
    ${catkin_LIBRARIES} 
)
target_include_directories(ros_node 
    PRIVATE
    ${catkin_INCLUDE_DIRS}
)