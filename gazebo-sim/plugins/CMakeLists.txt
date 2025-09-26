project(quadrotor_dynamics_plugin)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Dynamics plugin
add_library(quadrotor_dynamics_plugin SHARED quadrotor_dynamics_plugin.cpp)
target_link_libraries(quadrotor_dynamics_plugin ${GAZEBO_LIBRARIES})

