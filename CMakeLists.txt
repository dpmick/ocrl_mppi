cmake_minimum_required(VERSION 3.16)

project(mppi)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(  Eigen3 REQUIRED )

include_directories(
  include 
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