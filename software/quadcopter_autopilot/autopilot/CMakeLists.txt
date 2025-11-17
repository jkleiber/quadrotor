
file(GLOB SOURCE_LIST
    CONFIGURE_DEPENDS
    "src/*.cpp")
file(GLOB HEADER_LIST
    CONFIGURE_DEPENDS
    "include/autopilot/*.h")

add_library(quadcopter-autopilot-autopilot
    STATIC
    ${SOURCE_LIST}
    ${HEADER_LIST})

target_include_directories(quadcopter-autopilot-autopilot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(quadcopter-autopilot-autopilot 
    PUBLIC 
    quadcopter-autopilot-attitude-control 
    quadcopter-autopilot-altitude-control
    kleiber-control-pid 
    kleiber-control-core)