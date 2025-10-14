
cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
file(GLOB HEADER_LIST
    CONFIGURE_DEPENDS
    "include/control_sim/*.h")

file(GLOB SOURCE_LIST
    CONFIGURE_DEPENDS
    "src/*.cpp")

# Create the control simulation library
add_library(control_sim_lib ${SOURCE_LIST} ${HEADER_LIST})
target_include_directories(control_sim_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(control_sim_lib PROPERTIES LINKER_LANGUAGE CXX)

# Inherit include directories
get_target_property(IMGUI_INCLUDES IMGUI INCLUDE_DIRECTORIES)

if(WIN32)
    get_target_property(EIGEN_INCLUDES Eigen3::Eigen INCLUDE_DIRECTORIES)
    target_include_directories(control_sim_lib PUBLIC ../../include ${IMGUI_INCLUDES} ${EIGEN_INCLUDES})
else()
    target_include_directories(control_sim_lib PUBLIC ../../include ${IMGUI_INCLUDES})
endif(WIN32)

# Link dependencies.
target_link_libraries(control_sim_lib PUBLIC Eigen3::Eigen IMGUI IMPLOT kleiber-dynamics)

# All users of this library will need at least C++17
target_compile_features(control_sim_lib PUBLIC cxx_std_17)
