// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2018 Jérémie Dumas <jeremie.dumas@ens-lyon.org>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
////////////////////////////////////////////////////////////////////////////////
//#include "ImGuiMenu.h"
//#include "ImGuiHelpers.h"
#include <igl/project.h>
#include "ImGuiHelpers.h"

#include "ImGuiMenu.h"
//#include "../imgui.h"
#include "igl/opengl/glfw/imgui/imgui_impl_glfw.h"
#include "igl/opengl/glfw/imgui/imgui_impl_opengl3.h"

//#include <imgui_fonts_droid_sans.h>
//#include <GLFW/glfw3.h>
#include <iostream>
////////////////////////////////////////////////////////////////////////////////

namespace igl
{
namespace opengl
{
namespace glfw
{
namespace imgui
{

IGL_INLINE void ImGuiMenu::init(Display* disp)
{
  // Setup ImGui binding
  if (disp->window)
  {
    IMGUI_CHECKVERSION();
    if (!context_)
    {
      // Single global context by default, but can be overridden by the user
      static ImGuiContext * __global_context = ImGui::CreateContext();
      context_ = __global_context;
    }
    const char* glsl_version = "#version 150";

    ImGui_ImplGlfw_InitForOpenGL(disp->window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.FrameRounding = 5.0f;
    reload_font();
  }
}

IGL_INLINE void ImGuiMenu::reload_font(int font_size)
{
  hidpi_scaling_ = hidpi_scaling();
  pixel_ratio_ = pixel_ratio();
  ImGuiIO& io = ImGui::GetIO();
  io.Fonts->Clear();
 // io.Fonts->AddFontFromMemoryCompressedTTF(droid_sans_compressed_data,
 //   droid_sans_compressed_size, font_size * hidpi_scaling_);
  io.FontGlobalScale = 1.0 / pixel_ratio_;
}

IGL_INLINE void ImGuiMenu::shutdown()
{
  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  // User is responsible for destroying context if a custom context is given
  // ImGui::DestroyContext(*context_);
}

IGL_INLINE bool ImGuiMenu::pre_draw()
{
  glfwPollEvents();

  // Check whether window dpi has changed
  float scaling = hidpi_scaling();
  if (std::abs(scaling - hidpi_scaling_) > 1e-5)
  {
    reload_font();
    ImGui_ImplOpenGL3_DestroyDeviceObjects();
  }

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  return false;
}

IGL_INLINE bool ImGuiMenu::post_draw()
{
  //draw_menu(viewer,core);
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  return false;
}

IGL_INLINE void ImGuiMenu::post_resize(int width, int height)
{
  if (context_)
  {
    ImGui::GetIO().DisplaySize.x = float(width);
    ImGui::GetIO().DisplaySize.y = float(height);
  }
}

// Mouse IO
IGL_INLINE bool ImGuiMenu::mouse_down(GLFWwindow* window, int button, int modifier)
{
  ImGui_ImplGlfw_MouseButtonCallback(window, button, GLFW_PRESS, modifier);
  return ImGui::GetIO().WantCaptureMouse;
}

IGL_INLINE bool ImGuiMenu::mouse_up(GLFWwindow* window,int button, int modifier)
{
  //return ImGui::GetIO().WantCaptureMouse;
  // !! Should not steal mouse up
  return false;
}

IGL_INLINE bool ImGuiMenu::mouse_move(GLFWwindow* window,int mouse_x, int mouse_y)
{
  return ImGui::GetIO().WantCaptureMouse;
}

IGL_INLINE bool ImGuiMenu::mouse_scroll(GLFWwindow* window,float delta_y)
{
  ImGui_ImplGlfw_ScrollCallback(window, 0.f, delta_y);
  return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
IGL_INLINE bool ImGuiMenu::key_pressed(GLFWwindow* window,unsigned int key, int modifiers)
{
  ImGui_ImplGlfw_CharCallback(nullptr, key);
  return ImGui::GetIO().WantCaptureKeyboard;
}

IGL_INLINE bool ImGuiMenu::key_down(GLFWwindow* window, int key, int modifiers)
{
  ImGui_ImplGlfw_KeyCallback(window, key, 0, GLFW_PRESS, modifiers);
  return ImGui::GetIO().WantCaptureKeyboard;
}

IGL_INLINE bool ImGuiMenu::key_up(GLFWwindow* window,int key, int modifiers)
{
  ImGui_ImplGlfw_KeyCallback(window, key, 0, GLFW_RELEASE, modifiers);
  return ImGui::GetIO().WantCaptureKeyboard;
}

// Draw menu
IGL_INLINE void ImGuiMenu::draw_menu(igl::opengl::glfw::Viewer* viewer, std::vector<igl::opengl::ViewerCore>& core)
{
  // Text labels
  draw_labels_window(viewer,&core[1]);

  // Viewer settings
  if (callback_draw_viewer_window) { callback_draw_viewer_window(); }
  else { draw_viewer_window(viewer,core); }

  // Other windows
  if (callback_draw_custom_window) { callback_draw_custom_window(); }
  else { draw_custom_window(); }
}

IGL_INLINE void ImGuiMenu::draw_viewer_window(igl::opengl::glfw::Viewer* viewer, std::vector<igl::opengl::ViewerCore>& core)
{
  //float menu_width = 180.f * menu_scaling();
  //ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiSetCond_FirstUseEver);
  //ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiSetCond_FirstUseEver);
  //ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f), ImVec2(menu_width, -1.0f));
  //bool _viewer_menu_visible = true;

  bool open = false;
  //ImGui::ShowDemoWindow(&open);
  
}


IGL_INLINE void ImGuiMenu::draw_viewer_menu(igl::opengl::glfw::Viewer *viewer, std::vector<igl::opengl::ViewerCore>& core)
{
    bool* p_open = NULL;
    static bool no_titlebar = false;
    static bool no_scrollbar = false;
    static bool no_menu = true;
    static bool no_move = true;
    static bool no_resize = false;
    static bool no_collapse = false;
    static bool no_close = false;
    static bool no_nav = false;
    static bool no_background = false;
    static bool no_bring_to_front = false;
    static bool startMenu = true;
    static int clickedStart = 0;
    static int clickedExit = 0;
    static int clicked = 0;

    ImGuiWindowFlags window_flags = 0;
    if (no_titlebar)        window_flags |= ImGuiWindowFlags_NoTitleBar;
    if (no_scrollbar)       window_flags |= ImGuiWindowFlags_NoScrollbar;
    if (!no_menu)           window_flags |= ImGuiWindowFlags_MenuBar;
    if (no_move)            window_flags |= ImGuiWindowFlags_NoMove;
    if (no_resize)          window_flags |= ImGuiWindowFlags_NoResize;
    if (no_collapse)        window_flags |= ImGuiWindowFlags_NoCollapse;
    if (no_nav)             window_flags |= ImGuiWindowFlags_NoNav;
    if (no_background)      window_flags |= ImGuiWindowFlags_NoBackground;
    if (no_bring_to_front)  window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus;

    if (startMenu)
    {
        ImGui::SetNextWindowPos(ImVec2(350, 250), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(500, 300), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("3D Snake", p_open, window_flags))
        {
            ImGui::End();
            return;
        }
        ImGui::SetWindowFontScale(1.8);
        ImGui::Text("     Welcome to the Game :)", IMGUI_VERSION);
        ImGui::Text("Move Snake: ^,v,<,>", IMGUI_VERSION);
        ImGui::Text("Toggle Skinning: S,s", IMGUI_VERSION);
        ImGui::Text("Score 50 Points To Clear the Level", IMGUI_VERSION);
        static int clickedStart = 0;
        static int clickedExit = 0;
        if (ImGui::Button("START"))
            clickedStart++;
        if (clickedStart & 1)
        {
            viewer->levelWindow = true;
            startMenu = false;
        }
        ImGui::End();

    }

    if (viewer->levelWindow)
    {
        ImGui::SetNextWindowPos(ImVec2(1, 1), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(300, 800), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("  Level: "+viewer->level, p_open, window_flags))
        {
            ImGui::End();
            return;
        }
        if (!viewer->finishLevel)
        {
            ImGui::SetWindowFontScale(2.3);
            ImGui::Text("Score: %d", viewer->score);
        }
        else
            viewer->levelWindow = false;
        ImGui::End();

    }
    if (viewer->finishLevel)
    {
        ImGui::SetNextWindowPos(ImVec2(350, 250), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(500, 300), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("3D Snake", p_open, window_flags))
        {

            ImGui::End();
            return;
        }
        ImGui::SetWindowFontScale(1.8);
        ImGui::Text("Congragulations! you cleared level: %d",viewer->level);
        ImGui::Text("Score 50 Points To Clear the Next Level", IMGUI_VERSION);
        if (ImGui::Button("CONTINUE"))
            clicked++;
        if (clicked & 1)
        {
            viewer->levelWindow = true;
            viewer->finishLevel = false;
            viewer->level++;
            viewer->score = 0;
            clicked = 0;

        }


        ImGui::End();
    }
}

IGL_INLINE void ImGuiMenu::draw_labels_window(igl::opengl::glfw::Viewer* viewer, const igl::opengl::ViewerCore* core)
{
  // Text labels
  ImGui::SetNextWindowPos(ImVec2(0,0), ImGuiSetCond_Always);
  ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiSetCond_Always);
  bool visible = true;
  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0,0,0,0));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
  ImGui::Begin("ViewerLabels", &visible,
      ImGuiWindowFlags_NoTitleBar
      | ImGuiWindowFlags_NoResize
      | ImGuiWindowFlags_NoMove
      | ImGuiWindowFlags_NoScrollbar
      | ImGuiWindowFlags_NoScrollWithMouse
      | ImGuiWindowFlags_NoCollapse
      | ImGuiWindowFlags_NoSavedSettings
      | ImGuiWindowFlags_NoInputs);
  for (const auto & data : viewer->data_list)
  {
    draw_labels(data,core);
  }
  ImGui::End();
  ImGui::PopStyleColor();
  ImGui::PopStyleVar();
}

IGL_INLINE void ImGuiMenu::draw_labels(const igl::opengl::ViewerData& data, const igl::opengl::ViewerCore* core)
{
  if (data.show_vertid)
  {
    for (int i = 0; i < data.V.rows(); ++i)
    {
      draw_text(
        data.V.row(i), 
        data.V_normals.row(i), 
        std::to_string(i),
        core,data.label_color);
    }
  }

  if (data.show_faceid)
  {
    for (int i = 0; i < data.F.rows(); ++i)
    {
      Eigen::RowVector3d p = Eigen::RowVector3d::Zero();
      for (int j = 0; j < data.F.cols(); ++j)
      {
        p += data.V.row(data.F(i,j));
      }
      p /= (double) data.F.cols();

      draw_text(
        p, 
        data.F_normals.row(i), 
        std::to_string(i),
        core,data.label_color);
    }
  }

  if (data.labels_positions.rows() > 0)
  {
    for (int i = 0; i < data.labels_positions.rows(); ++i)
    {
      draw_text(
        data.labels_positions.row(i), 
        Eigen::Vector3d(0.0,0.0,0.0),
        data.labels_strings[i],
        core,data.label_color);
    }
  }
}

IGL_INLINE void ImGuiMenu::draw_text(
  Eigen::Vector3d pos, 
  Eigen::Vector3d normal, 
  const std::string &text,
  const igl::opengl::ViewerCore* core,
  const Eigen::Vector4f color)
{
  pos += normal * 0.005f * core[1].object_scale;
  Eigen::Vector3f coord = igl::project(Eigen::Vector3f(pos.cast<float>()),
    core[1].view, core[1].proj, core[1].viewport);

  // Draw text labels slightly bigger than normal text
  ImDrawList* drawList = ImGui::GetWindowDrawList();
  drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.2,
      ImVec2(coord[0]/pixel_ratio_, (core[1].viewport[3] - coord[1])/pixel_ratio_),
      ImGui::GetColorU32(ImVec4(
        color(0),
        color(1),
        color(2),
        color(3))),
      &text[0], &text[0] + text.size());
}

IGL_INLINE float ImGuiMenu::pixel_ratio()
{
  // Computes pixel ratio for hidpi devices
  int buf_size[2];
  int win_size[2];
  GLFWwindow* window = glfwGetCurrentContext();
  glfwGetFramebufferSize(window, &buf_size[0], &buf_size[1]);
  glfwGetWindowSize(window, &win_size[0], &win_size[1]);
  return (float) buf_size[0] / (float) win_size[0];
}

IGL_INLINE float ImGuiMenu::hidpi_scaling()
{
  // Computes scaling factor for hidpi devices
  float xscale, yscale;
  GLFWwindow* window = glfwGetCurrentContext();
  glfwGetWindowContentScale(window, &xscale, &yscale);
  return 0.5 * (xscale + yscale);
}

} // end namespace
} // end namespace
} // end namespace
} // end namespace
