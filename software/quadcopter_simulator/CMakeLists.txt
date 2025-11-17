
cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
file(GLOB HEADER_LIST
    CONFIGURE_DEPENDS
    "include/quadcopter_simulator/*.h")

file(GLOB SOURCE_LIST
    CONFIGURE_DEPENDS
    "src/*.cpp")

# Create the control simulation library
add_library(quadcopter-simulator ${SOURCE_LIST} ${HEADER_LIST})
target_include_directories(quadcopter-simulator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(quadcopter-simulator PROPERTIES LINKER_LANGUAGE CXX)

# Inherit include directories
get_target_property(IMGUI_INCLUDES IMGUI INCLUDE_DIRECTORIES)

if(WIN32)
    get_target_property(EIGEN_INCLUDES Eigen3::Eigen INCLUDE_DIRECTORIES)
    target_include_directories(quadcopter-simulator PUBLIC ../../include ${EIGEN_INCLUDES})
else()
endif(WIN32)

# Link dependencies.
target_link_libraries(quadcopter-simulator PUBLIC Eigen3::Eigen kleiber-dynamics)

# All users of this library will need at least C++17
target_compile_features(quadcopter-simulator PUBLIC cxx_std_17)
