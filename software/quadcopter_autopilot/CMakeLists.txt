
add_subdirectory(attitude_control)

add_library(quadcopter-autopilot INTERFACE)
target_link_libraries(quadcopter-autopilot INTERFACE quadcopter-autopilot-attitude-control)