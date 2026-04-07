// LiberaApp — reusable GLFW + ImGui bootstrap for Libera tools
//
// Handles: GLFW init, window creation, DPI-aware font loading (Roboto +
// ForkAwesome at three sizes), the Libera dark theme, and the per-frame
// begin/end/render boilerplate.

#pragma once

struct GLFWwindow;
struct ImFont;

struct LiberaAppConfig {
    const char* title       = "Libera App";
    int         width       = 1024;
    int         height      = 768;
    int         windowX     = -1;   // -1 = let OS decide
    int         windowY     = -1;
};

struct LiberaApp {
    GLFWwindow* window   = nullptr;
    float       dpiScale = 1.0f;
    ImFont*     fontBase   = nullptr;   // 16 px Roboto Medium + icons
    ImFont*     fontMedium = nullptr;   // 20 px Roboto Bold   + icons
    ImFont*     fontLarge  = nullptr;   // 28 px Roboto Bold   + icons

    // Initialise GLFW, create a window, set up ImGui + fonts + theme.
    // Returns false on failure (error printed to stderr).
    bool init(const LiberaAppConfig& config);

    // Call at the top of your loop.  Returns false when the window should close.
    bool beginFrame();

    // Call at the end of your loop (renders ImGui, swaps buffers).
    void endFrame();

    // Tear everything down (ImGui backends, GLFW).
    void shutdown();

    // Convenience: read back current window geometry (for settings persistence).
    void getWindowGeometry(int& x, int& y, int& w, int& h) const;
};
