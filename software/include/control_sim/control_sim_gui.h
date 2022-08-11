
#pragma once

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl.h"
#include "implot.h"

#include <SDL.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL_opengles2.h>
#else
#include <SDL_opengl.h>
#endif

#include <iostream>
#include <stdio.h>

#include "control_sim/events.h"

class GuiManager
{
public:
    GuiManager(GuiEvents *gui_events) : gui_events_(gui_events) {}

    bool InitGui();
    bool UpdateGui();
    bool Render();
    bool CloseGui();

    ~GuiManager() { CloseGui(); }

private:
    SDL_Window *window;

    SDL_GLContext gl_context;

    GuiEvents *const gui_events_;
};