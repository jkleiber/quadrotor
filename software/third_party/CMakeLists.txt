# =================== Eigen ====================
add_subdirectory(eigen3)

# =================== SDL2 ====================
set(SDL2_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDL2)
add_subdirectory(${SDL2_DIR})

# =================== IMGUI ===================
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
add_library(IMGUI STATIC)

target_sources(IMGUI
    PRIVATE
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui.cpp

    PRIVATE
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdl.cpp
)

target_include_directories(IMGUI
    PUBLIC ${IMGUI_DIR}
    PUBLIC ${IMGUI_DIR}/backends
    PUBLIC ${SDL2_DIR}/include
)

# OpenGL
find_package(OpenGL REQUIRED)
target_link_libraries(IMGUI PUBLIC SDL2-static ${CMAKE_DL_LIBS} ${OPENGL_LIBRARIES})

# =================== ImPlot ===================
set(IMPLOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/implot)
add_library(IMPLOT STATIC)

target_sources(IMPLOT
    PRIVATE
    ${IMPLOT_DIR}/implot_demo.cpp
    ${IMPLOT_DIR}/implot_items.cpp
    ${IMPLOT_DIR}/implot.cpp
)

target_include_directories(IMPLOT
    PUBLIC ${IMPLOT_DIR}/
)

target_link_libraries(IMPLOT PUBLIC IMGUI)