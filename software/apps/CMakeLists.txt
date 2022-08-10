# #####
# Compilation Options
OPTION(COMPILE_SIMULATION "Build the simulated robot code" ON)

# ================= Control Simulation ==================
add_executable(control_sim control_sim_main.cpp)
target_compile_features(control_sim PRIVATE cxx_std_17)
target_link_libraries(control_sim PRIVATE SDL2main control_sim_lib)

# ================= Flight Computer Code ==================
add_executable(quadrotor_fc main.cpp)

# ================= Gazebo Simulation ==================
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
    add_executable(quadrotor_fc_sim gazebo_sim_main.cpp)
    target_link_libraries(quadrotor_fc_sim
        PUBLIC
        ${GAZEBO_LIBRARIES}
        PRIVATE
    )
endif()