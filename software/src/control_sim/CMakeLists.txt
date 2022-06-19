project(control_sim)
cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

# add_subdirectory(kleiber_control)
# add_subdirectory(kleiber_guidance)
# add_subdirectory(kleiber_navigation)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(control_sim control_sim_main.cpp quadcopter_dynamics.cpp control_loop.cpp pid_controller.cpp sim_clock.cpp logging.cpp)
target_link_libraries(control_sim Eigen3::Eigen)