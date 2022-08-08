project(control_sim)
cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

file(GLOB HEADER_LIST
  CONFIGURE_DEPENDS 
  "${Quadrotor_SOURCE_DIR}/include/control_sim/*.h")

file(GLOB SOURCE_LIST
  CONFIGURE_DEPENDS 
  "${Quadrotor_SOURCE_DIR}/src/control_sim/*.cpp")

# Create the control simulation library
add_library(control_sim_lib ${SOURCE_LIST} ${HEADER_LIST})
set_target_properties(control_sim_lib PROPERTIES LINKER_LANGUAGE CXX)

# Inherit include directories
get_target_property(IMGUI_INCLUDES IMGUI INCLUDE_DIRECTORIES)
target_include_directories(control_sim_lib PUBLIC ../../include ${IMGUI_INCLUDES})

# This depends on Eigen and imgui
target_link_libraries(control_sim_lib PUBLIC Eigen3::Eigen IMGUI)

# All users of this library will need at least C++17
target_compile_features(control_sim_lib PUBLIC cxx_std_17)
