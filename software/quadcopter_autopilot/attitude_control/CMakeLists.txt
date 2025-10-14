
file(GLOB SOURCE_LIST
    CONFIGURE_DEPENDS
    "src/*.cpp")
file(GLOB HEADER_LIST
    CONFIGURE_DEPENDS
    "include/attitude_control/*.h")

add_library(quadcopter-autopilot-attitude-control
    STATIC
    ${SOURCE_LIST}
    ${HEADER_LIST})

target_include_directories(quadcopter-autopilot-attitude-control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(quadcopter-autopilot-attitude-control PUBLIC kleiber-control-pid kleiber-control-core)