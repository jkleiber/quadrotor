project(auto_mobile_robot)

cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(CheckIncludeFiles)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party 
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    )

add_subdirectory(src)


######
# Compilation Options
OPTION(COMPILE_SIMULATION "Build the simulated robot code" ON)


# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)



# Robot Code
add_executable(auto_quadrotor src/main.cpp )
# target_link_libraries(auto_mobile_robot 
#     PUBLIC 
#         kleiber_guidance 
#         kleiber_navigation 
#         kleiber_control 
#     PRIVATE 
#         ${SERIAL_LIBRARIES} 
#         ${CMAKE_THREAD_LIBS_INIT})



###################
#   Simulation
###################
if(COMPILE_SIMULATION)
    # Gazebo Packages
    find_package(gazebo REQUIRED)
    include_directories(${GAZEBO_INCLUDE_DIRS})
    link_directories(${GAZEBO_LIBRARY_DIRS})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

    # C++17
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)

    # Simulator Code
    add_executable(auto_quadrotor_sim 
        src/sim_main.cpp
        )
    target_link_libraries(auto_quadrotor_sim
        PUBLIC  
            ${GAZEBO_LIBRARIES}
        PRIVATE 
        )
endif()


###########################
#         Testing
###########################

