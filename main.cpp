// Libera Lab — laser controller service & test utility
//
// Cross-platform (macOS / Windows / Linux) using:
//   GLFW + OpenGL3 for windowing
//   Dear ImGui for the UI
//   libera-core for controller discovery and output

#include "libera.h"
#include "libera/core/ThreadUtils.hpp"
#include "LiberaApp.h"
#include "LiberaPluginsWindow.h"
#include "LiberaFileDialog.h"
#include "OscilloscopeWindow.h"

#include "imgui.h"
#include "fonts/IconsForkAwesome.h"
#include "ILDAParser.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <shlobj.h>
#include <windows.h>
#else
#include <dirent.h>
#include <pwd.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

using namespace libera;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// App state
// ---------------------------------------------------------------------------

static constexpr float PI = 3.14159265358979323846f;
static constexpr float TAU = 2.0f * PI;
static constexpr float MAX_SCAN_ANGLE = 60.0f;

struct SimResult {
    std::vector<float> x;  // simulated X positions (degrees)
    std::vector<float> y;  // simulated Y positions (degrees)
};

struct ScannerSim {
    // PD gains (galvo servos are primarily PD — no meaningful integral needed)
    float kpM = 1605.0f;  // proportional in millions
    float kd = 62000.0f;  // derivative (damping)

    // Plant parameters
    float inertia = 1.0f;      // moment of inertia (normalised)
    float friction = 200.0f;   // viscous damping coefficient

    // Simulation resolution
    float simRate = 150000.0f; // internal sim rate (pps)
};

static ScannerSim scannerSim;

static constexpr int EQ_NUM_BANDS = 24;

struct ScanStats {
    float maxSpeed = 0.0f;  // k°/s
    float maxAccel = 0.0f;  // °/ms² (peak)
    float rmsAccel = 0.0f;  // °/ms² (RMS)
    float medianAccel = 0.0f; // °/ms² (median)
    float rmsSpeed = 0.0f;  // k°/s (RMS velocity — scanner strain)
    std::size_t pointCount = 0;
};

struct SpectrumBands {
    float magnitude[EQ_NUM_BANDS] = {};
    float freqCenter[EQ_NUM_BANDS] = {};
    float maxMagnitude = 0.0f;
};

static constexpr int NUM_PATTERNS = 4;
static const char* patternNames[NUM_PATTERNS] = {
    "White Square", "Rainbow Circle", "Orientation", "Hot Corners"
};

enum class OutputMode { Shape, Point, Pattern };

static constexpr int NUM_POINT_PATTERNS = 5;
static const char* pointPatternNames[NUM_POINT_PATTERNS] = {
    "Single", "2 Vert", "2 Horiz", "4 Grid", "8 Row"
};

struct ILDAPattern {
    std::string name;             // filename without extension
    std::string path;             // full path
    std::vector<std::vector<ILDAPoint>> frames; // all frames (single frame for static files)
    float fps = 30.0f;            // playback frame rate for multi-frame files
    bool builtin = false;         // true = bundled at startup, cannot be deleted from UI
};

struct ControllerEntry {
    std::string id;
    std::string label;
    std::string type;
    std::uint32_t maxPointRate = 0;
    core::ControllerUsageState usageState = core::ControllerUsageState::Unknown;
    bool enabled = false;
    bool connecting = false;
    std::shared_ptr<core::LaserController> controller;
    std::future<std::shared_ptr<core::LaserController>> connectFuture;
};

struct DiscoveredInfo {
    std::string id;
    std::string label;
    std::string type;
    std::uint32_t maxPointRate = 0;
    core::ControllerUsageState usageState = core::ControllerUsageState::Unknown;
};

struct AppState {
    bool armed = false;
    float brightness = 10.0f;
    float red = 100.0f, green = 100.0f, blue = 100.0f;
    bool redEnabled = true, greenEnabled = true, blueEnabled = true;
    int outputMode = 0; // 0=Shape, 1=Point, 2=Pattern
    int patternIndex = 0;
    int pointPatternIndex = 0; // 0=single, 1=2V, 2=2H, 3=4grid, 4=8row
    float dutyCycle = 25.0f; // point mode duty cycle %
    int pointRateIndex = 0;
    int customPointRate = 30000;
    float outputSize = 50.0f;
    float outputX = 0.0f;
    float outputY = 0.0f;

    // ILDA patterns
    std::vector<ILDAPattern> ildaPatterns;
    int selectedIldaPattern = 0;
    bool singleColour = false;

    // ILDA animation playback (for multi-frame files)
    int ildaFrameIndex = 0;
    bool ildaPlaying = true;
    bool ildaLoop = true;
    std::chrono::steady_clock::time_point ildaLastAdvance = std::chrono::steady_clock::now();

    // Window geometry (saved/restored)
    int windowWidth = 1020;
    int windowHeight = 890;
    int windowX = -1; // -1 = let OS decide
    int windowY = -1;

    bool flipX = false;
    bool flipY = false;
    int orientation = 0;
    float scannerSync = 2.0f; // in 1/10,000s units (0.1ms). Default 2.0 = 0.2ms

    bool draggingOutput = false;
    bool resizingOutput = false;
    int resizingCorner = -1;
    bool previewZoomed = false;
    int simOverlay = 0; // 0=source only, 1=source+sim, 2=sim only
    bool showAdvanced = false;

    // Cached per-frame analysis (computed once per loop iteration)
    core::Frame previewFrame;  // after single colour, before brightness — for preview
    core::Frame currentFrame;  // fully processed — for output and stats
    SimResult simResult;
    ScanStats scanStats;
    SpectrumBands spectrum;

    std::set<std::string> savedEnabledControllers;

    System liberaSystem;
    std::vector<ControllerEntry> controllers;
    std::thread discoveryThread;
    std::atomic<bool> discoveryRunning{false};
    std::atomic<bool> discoveryFinished{false};
    std::mutex discoveredMutex;
    std::vector<DiscoveredInfo> latestDiscovered;
    std::atomic<bool> discoveryResultReady{false};
    std::atomic<bool> discoveryRequested{false};
    std::mutex discoveryCvMutex;
    std::condition_variable discoveryCv;

    struct PointRatePreset {
        const char* label;
        const char* shortLabel;
        int value;
    };
    static constexpr PointRatePreset pointRatePresets[] = {
        {"12,000 pps", "12k",  12000},
        {"18,000 pps", "18k",  18000},
        {"20,000 pps", "20k",  20000},
        {"24,000 pps", "24k",  24000},
        {"30,000 pps", "30k",  30000},
        {"40,000 pps", "40k",  40000},
        {"50,000 pps", "50k",  50000},
    };
    static constexpr int numPresets = sizeof(pointRatePresets) / sizeof(pointRatePresets[0]);
    static constexpr int customIndex = numPresets;

    int effectivePointRate() const {
        if (pointRateIndex >= 0 && pointRateIndex < numPresets)
            return pointRatePresets[pointRateIndex].value;
        return customPointRate;
    }

    // Effective RGB considering brightness, per-channel level, and enable
    void effectiveRGB(float& r, float& g, float& b) const {
        float br = brightness / 100.0f;
        r = redEnabled   ? br * (red   / 100.0f) : 0.0f;
        g = greenEnabled ? br * (green / 100.0f) : 0.0f;
        b = blueEnabled  ? br * (blue  / 100.0f) : 0.0f;
    }

    void resetOutput() {
        outputSize = 50.0f;
        outputX = 0.0f;
        outputY = 0.0f;
    }
};

// ---------------------------------------------------------------------------
// Settings persistence
// ---------------------------------------------------------------------------

static std::string getSettingsDir() {
    std::string dir;
#ifdef _WIN32
    char path[MAX_PATH];
    if (SHGetFolderPathA(nullptr, CSIDL_APPDATA, nullptr, 0, path) == S_OK) {
        dir = std::string(path) + "\\LiberaLab";
    } else {
        dir = ".";
    }
#elif defined(__APPLE__)
    const char* home = getenv("HOME");
    if (!home) home = getpwuid(getuid())->pw_dir;
    dir = std::string(home) + "/Library/Application Support/LiberaLab";
#else
    const char* xdg = getenv("XDG_CONFIG_HOME");
    if (xdg) {
        dir = std::string(xdg) + "/LiberaLab";
    } else {
        const char* home = getenv("HOME");
        if (!home) home = getpwuid(getuid())->pw_dir;
        dir = std::string(home) + "/.config/LiberaLab";
    }
#endif
    return dir;
}

static std::string getSettingsPath() { return getSettingsDir() + "/settings.ini"; }

static void ensureSettingsDir() {
    std::string dir = getSettingsDir();
#ifdef _WIN32
    CreateDirectoryA(dir.c_str(), nullptr);
#else
    mkdir(dir.c_str(), 0755);
#endif
}

// User patterns live alongside settings, so they persist per-user across app
// updates and don't collide with the bundled read-only patterns folder.
// Windows: %APPDATA%\LiberaLab\patterns
// macOS:   ~/Library/Application Support/LiberaLab/patterns
// Linux:   $XDG_CONFIG_HOME/LiberaLab/patterns (or ~/.config/LiberaLab/patterns)
static std::string getUserPatternsDir() {
    return getSettingsDir() + "/patterns";
}

static void ensureUserPatternsDir() {
    ensureSettingsDir();
    std::error_code ec;
    std::filesystem::create_directories(getUserPatternsDir(), ec);
}

static void saveSettings(const AppState& state) {
    ensureSettingsDir();
    std::ofstream f(getSettingsPath());
    if (!f) return;
    f << "[LiberaLab]\n";
    f << "brightness=" << state.brightness << "\n";
    f << "red=" << state.red << "\n";
    f << "green=" << state.green << "\n";
    f << "blue=" << state.blue << "\n";
    f << "redEnabled=" << (state.redEnabled ? 1 : 0) << "\n";
    f << "greenEnabled=" << (state.greenEnabled ? 1 : 0) << "\n";
    f << "blueEnabled=" << (state.blueEnabled ? 1 : 0) << "\n";
    f << "outputMode=" << state.outputMode << "\n";
    f << "pattern=" << state.patternIndex << "\n";
    f << "pointPatternIndex=" << state.pointPatternIndex << "\n";
    f << "dutyCycle=" << state.dutyCycle << "\n";
    f << "pointRateIndex=" << state.pointRateIndex << "\n";
    f << "customPointRate=" << state.customPointRate << "\n";
    f << "outputSize=" << state.outputSize << "\n";
    f << "outputX=" << state.outputX << "\n";
    f << "outputY=" << state.outputY << "\n";
    f << "flipX=" << (state.flipX ? 1 : 0) << "\n";
    f << "flipY=" << (state.flipY ? 1 : 0) << "\n";
    f << "orientation=" << state.orientation << "\n";
    f << "scannerSync=" << state.scannerSync << "\n";
    f << "showAdvanced=" << (state.showAdvanced ? 1 : 0) << "\n";
    f << "simKpM=" << scannerSim.kpM << "\n";
    f << "simKd=" << scannerSim.kd << "\n";
    f << "simInertia=" << scannerSim.inertia << "\n";
    f << "simFriction=" << scannerSim.friction << "\n";
    f << "windowWidth=" << state.windowWidth << "\n";
    f << "windowHeight=" << state.windowHeight << "\n";
    f << "windowX=" << state.windowX << "\n";
    f << "windowY=" << state.windowY << "\n";
    for (auto& entry : state.controllers) {
        if (entry.enabled)
            f << "enabledController=" << entry.id << "\n";
    }
}

static void loadSettings(AppState& state) {
    std::ifstream f(getSettingsPath());
    if (!f) return;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '[' || line[0] == '#') continue;
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);
        if (key == "brightness") state.brightness = std::stof(val);
        else if (key == "red") state.red = std::stof(val);
        else if (key == "green") state.green = std::stof(val);
        else if (key == "blue") state.blue = std::stof(val);
        else if (key == "redEnabled") state.redEnabled = (std::stoi(val) != 0);
        else if (key == "greenEnabled") state.greenEnabled = (std::stoi(val) != 0);
        else if (key == "blueEnabled") state.blueEnabled = (std::stoi(val) != 0);
        else if (key == "outputMode") state.outputMode = std::stoi(val);
        else if (key == "pattern") state.patternIndex = std::stoi(val);
        else if (key == "pointPatternIndex") state.pointPatternIndex = std::stoi(val);
        else if (key == "dutyCycle") state.dutyCycle = std::stof(val);
        else if (key == "pointRateIndex") state.pointRateIndex = std::stoi(val);
        else if (key == "customPointRate") state.customPointRate = std::stoi(val);
        else if (key == "outputSize") state.outputSize = std::stof(val);
        else if (key == "outputX") state.outputX = std::stof(val);
        else if (key == "outputY") state.outputY = std::stof(val);
        else if (key == "flipX") state.flipX = (std::stoi(val) != 0);
        else if (key == "flipY") state.flipY = (std::stoi(val) != 0);
        else if (key == "orientation") state.orientation = std::stoi(val);
        else if (key == "scannerSync") state.scannerSync = std::stof(val);
        else if (key == "showAdvanced") state.showAdvanced = (std::stoi(val) != 0);
        else if (key == "simKpM") scannerSim.kpM = std::stof(val);
        else if (key == "simKd") scannerSim.kd = std::stof(val);
        else if (key == "simInertia") scannerSim.inertia = std::stof(val);
        else if (key == "simFriction") scannerSim.friction = std::stof(val);
        else if (key == "windowWidth") state.windowWidth = std::stoi(val);
        else if (key == "windowHeight") state.windowHeight = std::stoi(val);
        else if (key == "windowX") state.windowX = std::stoi(val);
        else if (key == "windowY") state.windowY = std::stoi(val);
        else if (key == "enabledController") state.savedEnabledControllers.insert(val);
    }
}

// ---------------------------------------------------------------------------
// Apply orientation transforms to a point
// ---------------------------------------------------------------------------

static void applyTransform(float& x, float& y, const AppState& state) {
    if (state.flipX) x = -x;
    if (state.flipY) y = -y;
    for (int r = 0; r < state.orientation; ++r) {
        float tmp = x;
        x = -y;
        y = tmp;
    }
}

// ---------------------------------------------------------------------------
// Frame generation
// ---------------------------------------------------------------------------

static core::Frame makeSquareFrame(float size, float cx, float cy) {
    core::Frame frame;
    const float s = size * 0.8f;
    constexpr int pointsPerSide = 100;
    frame.points.reserve(pointsPerSide * 4 + 1);
    const float corners[][2] = {
        {cx - s, cy - s}, {cx + s, cy - s},
        {cx + s, cy + s}, {cx - s, cy + s},
    };
    for (int side = 0; side < 4; ++side) {
        int next = (side + 1) % 4;
        for (int i = 0; i < pointsPerSide; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(pointsPerSide);
            float x = corners[side][0] + t * (corners[next][0] - corners[side][0]);
            float y = corners[side][1] + t * (corners[next][1] - corners[side][1]);
            frame.points.push_back({x, y, 1, 1, 1});
        }
    }
    // Close the loop
    frame.points.push_back(frame.points.front());
    return frame;
}

static core::Frame makeCircleFrame(float size, float cx, float cy) {
    core::Frame frame;
    const float radius = size * 0.8f;
    constexpr int pointCount = 400;
    frame.points.reserve(pointCount);
    for (int i = 0; i < pointCount; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(pointCount);
        float angle = t * TAU;
        // Rainbow: hue cycles around the circle
        float h = t * 6.0f;
        int hi = static_cast<int>(h) % 6;
        float f = h - std::floor(h);
        float r, g, b;
        switch (hi) {
            case 0: r = 1; g = f;     b = 0;     break;
            case 1: r = 1 - f; g = 1; b = 0;     break;
            case 2: r = 0; g = 1;     b = f;     break;
            case 3: r = 0; g = 1 - f; b = 1;     break;
            case 4: r = f; g = 0;     b = 1;     break;
            default:r = 1; g = 0;     b = 1 - f; break;
        }
        frame.points.push_back({
            cx + radius * std::cos(angle),
            cy + radius * std::sin(angle),
            r, g, b
        });
    }
    return frame;
}

// RGBW Square: white top, red left, green bottom, blue right
static core::Frame makeRGBWSquareFrame(float size, float cx, float cy) {
    core::Frame frame;
    const float s = size * 0.8f;
    constexpr int pointsPerSide = 100;
    frame.points.reserve(pointsPerSide * 4);

    // Corners: TL, TR, BR, BL
    const float corners[][2] = {
        {cx - s, cy - s}, {cx + s, cy - s},
        {cx + s, cy + s}, {cx - s, cy + s},
    };
    // Sides: 0=TL→TR (low Y, screen bottom), 1=TR→BR (right),
    //        2=BR→BL (high Y, screen top), 3=BL→TL (left)
    // Clockwise from top: WHITE, RED, GREEN, BLUE
    struct SideCol { float r, g, b; };
    const SideCol sideCols[] = {
        {0, 1, 0},   // side 0 (bottom): green
        {1, 0, 0},   // side 1 (right): red
        {1, 1, 1},   // side 2 (top): white
        {0, 0, 1},   // side 3 (left): blue
    };

    for (int side = 0; side < 4; ++side) {
        int next = (side + 1) % 4;
        for (int i = 0; i < pointsPerSide; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(pointsPerSide);
            float x = corners[side][0] + t * (corners[next][0] - corners[side][0]);
            float y = corners[side][1] + t * (corners[next][1] - corners[side][1]);
            frame.points.push_back({x, y, sideCols[side].r, sideCols[side].g, sideCols[side].b});
        }
    }
    frame.points.push_back(frame.points.front());
    return frame;
}

// Hot Corners Square: fast along edges, lingers at corners
static core::Frame makeHotCornersFrame(float size, float cx, float cy) {
    core::Frame frame;
    const float s = size * 0.8f;
    constexpr int cornerDwell = 30;   // points dwelling at each corner
    constexpr int edgePoints = 20;    // points along each edge (fast transit)
    constexpr int totalPerSide = cornerDwell + edgePoints;
    frame.points.reserve(totalPerSide * 4);

    const float corners[][2] = {
        {cx - s, cy - s}, {cx + s, cy - s},
        {cx + s, cy + s}, {cx - s, cy + s},
    };

    for (int side = 0; side < 4; ++side) {
        int next = (side + 1) % 4;
        // Dwell at this corner
        for (int i = 0; i < cornerDwell; ++i) {
            frame.points.push_back({corners[side][0], corners[side][1], 1, 1, 1});
        }
        // Move quickly along edge
        for (int i = 0; i < edgePoints; ++i) {
            float t = static_cast<float>(i + 1) / static_cast<float>(edgePoints);
            float x = corners[side][0] + t * (corners[next][0] - corners[side][0]);
            float y = corners[side][1] + t * (corners[next][1] - corners[side][1]);
            frame.points.push_back({x, y, 1, 1, 1});
        }
    }
    // Already closes: last edge ends at corner 0 = first dwell point
    return frame;
}

// Generate point positions for a given point pattern
struct PointDot { float x, y, r, g, b; };

static std::vector<PointDot> getPointPositions(int patternIndex, float cx, float cy, float spacing) {
    std::vector<PointDot> pts;
    float s = spacing * 0.3f; // spacing between dots
    switch (patternIndex) {
        case 0: // Single
            pts.push_back({cx, cy, 1, 1, 1});
            break;
        case 1: // 2 Vertical
            pts.push_back({cx, cy - s, 1, 1, 1});
            pts.push_back({cx, cy + s, 1, 1, 1});
            break;
        case 2: // 2 Horizontal
            pts.push_back({cx - s, cy, 1, 1, 1});
            pts.push_back({cx + s, cy, 1, 1, 1});
            break;
        case 3: // 4 Grid — RGBW: top-left=green, top-right=red, bottom-right=white, bottom-left=blue
            pts.push_back({cx - s, cy - s, 0, 1, 0});  // bottom-left (screen): green
            pts.push_back({cx + s, cy - s, 1, 0, 0});  // bottom-right (screen): red
            pts.push_back({cx + s, cy + s, 1, 1, 1});  // top-right (screen): white
            pts.push_back({cx - s, cy + s, 0, 0, 1});  // top-left (screen): blue
            break;
        case 4: // 8 Row — rainbow, left-to-right then right-to-left
        {
            float halfW = spacing * 0.8f;
            auto hueToRGB = [](float h) -> PointDot {
                float hh = h * 6.0f;
                int hi = static_cast<int>(hh) % 6;
                float f = hh - std::floor(hh);
                float r, g, b;
                switch (hi) {
                    case 0: r=1; g=f;   b=0;   break;
                    case 1: r=1-f; g=1; b=0;   break;
                    case 2: r=0; g=1;   b=f;   break;
                    case 3: r=0; g=1-f; b=1;   break;
                    case 4: r=f; g=0;   b=1;   break;
                    default:r=1; g=0;   b=1-f; break;
                }
                return {0, 0, r, g, b};
            };
            // Left to right
            for (int i = 0; i < 8; ++i) {
                float t = (static_cast<float>(i) / 7.0f - 0.5f) * 2.0f * halfW;
                auto dot = hueToRGB(static_cast<float>(i) / 8.0f);
                dot.x = cx + t; dot.y = cy;
                pts.push_back(dot);
            }
            // Right to left (skip endpoints to avoid double-dwell)
            for (int i = 6; i >= 1; --i) {
                float t = (static_cast<float>(i) / 7.0f - 0.5f) * 2.0f * halfW;
                auto dot = hueToRGB(static_cast<float>(i) / 8.0f);
                dot.x = cx + t; dot.y = cy;
                pts.push_back(dot);
            }
            break;
        }
        default:
            pts.push_back({cx, cy, 1, 1, 1});
            break;
    }
    return pts;
}

static core::Frame makePointFrame(int patternIndex,
                                   float cx, float cy, float size, float dutyCycle) {
    core::Frame frame;
    auto positions = getPointPositions(patternIndex, cx, cy, size);
    int numDots = static_cast<int>(positions.size());
    int dwellPerDot = std::max(4, 100 / numDots);
    int onPoints = std::max(1, static_cast<int>(std::round(dutyCycle / 100.0f * dwellPerDot)));
    int transitPoints = (numDots > 1) ? 20 : 0; // smooth blank path between dots

    frame.points.reserve((dwellPerDot + transitPoints) * numDots);
    for (int d = 0; d < numDots; ++d) {
        auto& dot = positions[d];

        // Dwell on this dot (with duty cycle)
        for (int i = 0; i < dwellPerDot; ++i) {
            if (i < onPoints) {
                frame.points.push_back({dot.x, dot.y, dot.r, dot.g, dot.b});
            } else {
                frame.points.push_back({dot.x, dot.y, 0, 0, 0});
            }
        }

        // Smooth blank transit to next dot
        if (numDots > 1) {
            auto& next = positions[(d + 1) % numDots];
            float nx = next.x, ny = next.y;
            for (int i = 0; i < transitPoints; ++i) {
                float t = static_cast<float>(i + 1) / static_cast<float>(transitPoints + 1);
                float tx = dot.x + t * (nx - dot.x);
                float ty = dot.y + t * (ny - dot.y);
                frame.points.push_back({tx, ty, 0, 0, 0});
            }
        }
    }
    return frame;
}

static core::Frame makeShapeFrame(int patternIndex, float size, float cx, float cy) {
    switch (patternIndex) {
        case 0: return makeSquareFrame(size, cx, cy);
        case 1: return makeCircleFrame(size, cx, cy);
        case 2: return makeRGBWSquareFrame(size, cx, cy);
        case 3: return makeHotCornersFrame(size, cx, cy);
    }
    return makeSquareFrame(size, cx, cy);
}

// ---------------------------------------------------------------------------
// ILDA pattern loading and frame conversion
// ---------------------------------------------------------------------------

static std::string getPatternsDir(const char* argv0) {
#ifdef __APPLE__
    // Inside an app bundle: look in Contents/Resources/patterns
    std::string exePath(argv0);
    auto lastSlash = exePath.find_last_of('/');
    std::string macosDir = (lastSlash != std::string::npos) ? exePath.substr(0, lastSlash) : ".";
    std::string resourceDir = macosDir + "/../Resources/patterns";
    // Fall back to next-to-executable for non-bundle builds
    struct stat st;
    if (stat(resourceDir.c_str(), &st) == 0 && S_ISDIR(st.st_mode))
        return resourceDir;
#endif
    // Default: patterns/ next to the executable
    std::string exePath2(argv0);
    auto lastSlash2 = exePath2.find_last_of("/\\");
    std::string dir = (lastSlash2 != std::string::npos) ? exePath2.substr(0, lastSlash2) : ".";
    return dir + "/patterns";
}

// Scan `dir` for .ild files and append to state.ildaPatterns.
// Does NOT clear existing entries — call once per source directory.
static void scanILDADir(AppState& state, const std::string& dir, bool builtin) {
#ifdef _WIN32
    WIN32_FIND_DATAA fd;
    HANDLE hFind = FindFirstFileA((dir + "\\*.ild").c_str(), &fd);
    if (hFind == INVALID_HANDLE_VALUE) return;
    do {
        std::string filename = fd.cFileName;
#else
    auto* d = opendir(dir.c_str());
    if (!d) return;
    struct dirent* entry;
    while ((entry = readdir(d)) != nullptr) {
        std::string filename = entry->d_name;
        if (filename.size() < 4) continue;
        std::string ext = filename.substr(filename.size() - 4);
        for (auto& c : ext) c = static_cast<char>(::tolower(c));
        if (ext != ".ild") continue;
#endif
        std::string fullPath = dir + "/" + filename;
        auto frames = ILDAParser::load(fullPath);
        if (!frames.empty() && !frames[0].empty()) {
            std::string name = filename;
            auto dot = name.find_last_of('.');
            if (dot != std::string::npos) name = name.substr(0, dot);

            ILDAPattern p;
            p.name = std::move(name);
            p.path = fullPath;
            p.frames = std::move(frames);
            p.builtin = builtin;
            state.ildaPatterns.push_back(std::move(p));
        }
#ifdef _WIN32
    } while (FindNextFileA(hFind, &fd));
    FindClose(hFind);
#else
    }
    closedir(d);
#endif
}

static void loadILDAPatterns(AppState& state, const std::string& bundledDir) {
    state.ildaPatterns.clear();
    scanILDADir(state, bundledDir, /*builtin=*/true);
    scanILDADir(state, getUserPatternsDir(), /*builtin=*/false);

    // Sort: built-ins first, then user patterns, each group alphabetically
    std::sort(state.ildaPatterns.begin(), state.ildaPatterns.end(),
              [](const ILDAPattern& a, const ILDAPattern& b) {
                  if (a.builtin != b.builtin) return a.builtin && !b.builtin;
                  return a.name < b.name;
              });
}

// Load a single .ild file at `srcPath`, copy it into the user patterns
// directory (so it persists across launches), and append it to the pattern
// list. Returns the index of the new pattern, or -1 on failure. If a pattern
// with the same filename already exists, finds a unique "foo (2).ild" name.
static int addILDAFile(AppState& state, const std::string& srcPath) {
    namespace fs = std::filesystem;

    // Parse first to validate before copying
    auto frames = ILDAParser::load(srcPath);
    if (frames.empty() || frames[0].empty()) return -1;

    fs::path src(srcPath);
    std::string stem = src.stem().string();
    std::string ext = src.extension().string();
    if (ext.empty()) ext = ".ild";

    ensureUserPatternsDir();
    fs::path destDir(getUserPatternsDir());
    fs::path dest = destDir / (stem + ext);

    // If a different file already occupies the target name, find a unique suffix
    std::error_code ec;
    int suffix = 2;
    while (fs::exists(dest, ec) && !fs::equivalent(src, dest, ec)) {
        dest = destDir / (stem + " (" + std::to_string(suffix++) + ")" + ext);
    }

    // Copy unless we're already pointing at the same file on disk
    if (!fs::equivalent(src, dest, ec)) {
        fs::copy_file(src, dest, fs::copy_options::overwrite_existing, ec);
        if (ec) return -1;
    }

    std::string destStr = dest.string();
    std::string finalName = dest.stem().string();

    // If this exact copy is already in the list, just refresh its frames
    for (std::size_t i = 0; i < state.ildaPatterns.size(); ++i) {
        if (state.ildaPatterns[i].path == destStr) {
            state.ildaPatterns[i].name = finalName;
            state.ildaPatterns[i].frames = std::move(frames);
            state.ildaPatterns[i].builtin = false;
            return static_cast<int>(i);
        }
    }

    ILDAPattern p;
    p.name = std::move(finalName);
    p.path = destStr;
    p.frames = std::move(frames);
    p.builtin = false;
    state.ildaPatterns.push_back(std::move(p));
    return static_cast<int>(state.ildaPatterns.size()) - 1;
}

// Convert ILDA points to a libera Frame, applying brightness/RGB and size/offset
static core::Frame makeILDAFrame(const std::vector<ILDAPoint>& ildaPoints,
                                  float size, float cx, float cy) {
    core::Frame frame;
    frame.points.reserve(ildaPoints.size());

    for (auto& ip : ildaPoints) {
        // ILDA coords: -32768..32767 → normalised -1..1, then scale by size and offset
        float nx = static_cast<float>(ip.x) / 32768.0f * size + cx;
        float ny = static_cast<float>(ip.y) / 32768.0f * size + cy;

        if (ip.blank) {
            frame.points.push_back({nx, ny, 0, 0, 0});
        } else {
            float ir = static_cast<float>((ip.color >> 16) & 0xFF) / 255.0f;
            float ig = static_cast<float>((ip.color >> 8) & 0xFF) / 255.0f;
            float ib = static_cast<float>(ip.color & 0xFF) / 255.0f;
            frame.points.push_back({nx, ny, ir, ig, ib});
        }
    }
    return frame;
}

// ---------------------------------------------------------------------------
// Draw a pattern into a drawlist at a given rect
// ---------------------------------------------------------------------------

// brightnessScale multiplies the frame colours for armed/disarmed dimming
static void drawPatternInRect(ImDrawList* drawList, int patternIndex,
                               float size, float cx, float cy,
                               ImVec2 rectPos, ImVec2 rectSize,
                               float brightnessScale = 1.0f,
                               bool flipYForPreview = false,
                               float viewScale = 1.0f, float viewOffX = 0.0f, float viewOffY = 0.0f) {
    core::Frame frame = makeShapeFrame(patternIndex, size, cx, cy);

    auto mapX = [&](float x) { return rectPos.x + ((x - viewOffX) * viewScale + 1.0f) * 0.5f * rectSize.x; };
    auto mapY = [&](float y) {
        if (flipYForPreview) y = -y;
        float vy = flipYForPreview ? -viewOffY : viewOffY;
        return rectPos.y + ((y - vy) * viewScale + 1.0f) * 0.5f * rectSize.y;
    };

    for (std::size_t i = 1; i < frame.points.size(); ++i) {
        auto& pt = frame.points[i];
        uint8_t cr = static_cast<uint8_t>(std::min(255.0f, pt.r * 255.0f * brightnessScale));
        uint8_t cg = static_cast<uint8_t>(std::min(255.0f, pt.g * 255.0f * brightnessScale));
        uint8_t cb = static_cast<uint8_t>(std::min(255.0f, pt.b * 255.0f * brightnessScale));
        ImU32 col = IM_COL32(cr, cg, cb, 255);

        drawList->AddLine(
            ImVec2(mapX(frame.points[i - 1].x), mapY(frame.points[i - 1].y)),
            ImVec2(mapX(frame.points[i].x),     mapY(frame.points[i].y)),
            col, 1.5f);
    }
}

// Draw ILDA points into a rect (for thumbnails and preview)
// scale/cx/cy apply in normalised space: coords are scaled then offset before mapping to rect
static void drawILDAInRect(ImDrawList* drawList, const std::vector<ILDAPoint>& points,
                            ImVec2 rectPos, ImVec2 rectSize, bool flipY = false,
                            float brightnessScale = 1.0f,
                            float scale = 1.0f, float cx = 0.0f, float cy = 0.0f,
                            float viewScale = 1.0f, float viewOffX = 0.0f, float viewOffY = 0.0f) {
    if (points.empty()) return;

    auto mapX = [&](float x) { return rectPos.x + ((x - viewOffX) * viewScale + 1.0f) * 0.5f * rectSize.x; };
    auto mapY = [&](float y) {
        if (flipY) y = -y;
        float vy = flipY ? -viewOffY : viewOffY;
        return rectPos.y + ((y - vy) * viewScale + 1.0f) * 0.5f * rectSize.y;
    };

    for (std::size_t i = 1; i < points.size(); ++i) {
        if (points[i].blank || points[i - 1].blank) continue;

        float x0 = static_cast<float>(points[i - 1].x) / 32768.0f * scale + cx;
        float y0 = static_cast<float>(points[i - 1].y) / 32768.0f * scale + cy;
        float x1 = static_cast<float>(points[i].x) / 32768.0f * scale + cx;
        float y1 = static_cast<float>(points[i].y) / 32768.0f * scale + cy;

        uint32_t c = points[i].color;
        uint8_t r = static_cast<uint8_t>(std::min(255.0f, ((c >> 16) & 0xFF) * brightnessScale));
        uint8_t g = static_cast<uint8_t>(std::min(255.0f, ((c >> 8) & 0xFF) * brightnessScale));
        uint8_t b = static_cast<uint8_t>(std::min(255.0f, (c & 0xFF) * brightnessScale));

        drawList->AddLine(
            ImVec2(mapX(x0), mapY(y0)),
            ImVec2(mapX(x1), mapY(y1)),
            IM_COL32(r, g, b, 255), 1.5f);
    }
}

// ---------------------------------------------------------------------------
// Galvo scanner PID simulation
// ---------------------------------------------------------------------------
// Models a single-axis galvo as a second-order plant (inertia + damping)
// driven by a PID controller. Returns the simulated actual positions.

static SimResult simulateScanner(const core::Frame& frame, float pointRate,
                                  const ScannerSim& sim) {
    SimResult result;
    std::size_t n = frame.points.size();
    if (n < 2 || pointRate <= 0) return result;

    constexpr float halfAngle = MAX_SCAN_ANGLE * 0.5f;
    float kp = sim.kpM * 1e6f;

    // Oversample: number of sim steps per output point
    int stepsPerPoint = std::max(1, (int)(sim.simRate / pointRate + 0.5f));
    float dt = 1.0f / (pointRate * (float)stepsPerPoint);

    std::size_t totalSamples = n * (std::size_t)stepsPerPoint;
    result.x.resize(totalSamples);
    result.y.resize(totalSamples);

    // Run a few warm-up loops so the servo state is settled for the
    // periodic frame (avoids transient from starting at rest)
    constexpr int warmupLoops = 3;

    // State per axis: position, velocity
    float posX = frame.points[0].x * halfAngle;
    float posY = frame.points[0].y * halfAngle;
    float velX = 0, velY = 0;

    auto stepAxis = [&](float pos, float vel,
                        float cmd, float dt_s,
                        float& outPos, float& outVel) {
        float err = cmd - pos;
        float torque = kp * err + sim.kd * (-vel);
        float accel = (torque - sim.friction * vel) / sim.inertia;
        vel += accel * dt_s;
        pos += vel * dt_s;

        outPos = pos;
        outVel = vel;
    };

    // Linearly interpolate command position for sub-steps
    auto cmdAt = [&](std::size_t idx, int sub) -> std::pair<float, float> {
        std::size_t next = (idx + 1) % n;
        float t = (float)sub / (float)stepsPerPoint;
        float cx = ((1.0f - t) * frame.points[idx].x + t * frame.points[next].x) * halfAngle;
        float cy = ((1.0f - t) * frame.points[idx].y + t * frame.points[next].y) * halfAngle;
        return {cx, cy};
    };

    // Warm-up: run through the frame several times without recording
    for (int loop = 0; loop < warmupLoops; ++loop) {
        for (std::size_t i = 0; i < n; ++i) {
            for (int s = 0; s < stepsPerPoint; ++s) {
                auto [cx, cy] = cmdAt(i, s);
                stepAxis(posX, velX, cx, dt, posX, velX);
                stepAxis(posY, velY, cy, dt, posY, velY);
            }
        }
    }

    // Record one full frame at full sim rate
    std::size_t idx = 0;
    for (std::size_t i = 0; i < n; ++i) {
        for (int s = 0; s < stepsPerPoint; ++s) {
            auto [cx, cy] = cmdAt(i, s);
            stepAxis(posX, velX, cx, dt, posX, velX);
            stepAxis(posY, velY, cy, dt, posY, velY);
            result.x[idx] = posX;
            result.y[idx] = posY;
            ++idx;
        }
    }

    return result;
}

// ---------------------------------------------------------------------------
// Scan stats calculation (from simulated scanner positions)
// ---------------------------------------------------------------------------

static ScanStats calculateScanStats(const SimResult& sim, float sampleRate) {
    ScanStats stats;
    std::size_t n = sim.x.size();
    stats.pointCount = n;
    if (n < 3 || sampleRate <= 0) return stats;

    float dt = 1.0f / sampleRate;

    float prevVx = 0, prevVy = 0;
    bool havePrevVel = false;
    float accelSumSq = 0.0f;
    float speedSumSq = 0.0f;
    std::vector<float> accelValues;
    accelValues.reserve(n);

    for (std::size_t i = 1; i < n; ++i) {
        float dx = sim.x[i] - sim.x[i - 1];
        float dy = sim.y[i] - sim.y[i - 1];

        float vx = dx / dt, vy = dy / dt;
        float speedSq = vx * vx + vy * vy;
        float speed = std::sqrt(speedSq);
        if (speed > stats.maxSpeed) stats.maxSpeed = speed;
        speedSumSq += speedSq;

        if (havePrevVel) {
            float ax = (vx - prevVx) / dt;
            float ay = (vy - prevVy) / dt;
            float accelSq = ax * ax + ay * ay;
            float accel = std::sqrt(accelSq);
            if (accel > stats.maxAccel) stats.maxAccel = accel;
            accelSumSq += accelSq;
            accelValues.push_back(accel);
        }

        prevVx = vx;
        prevVy = vy;
        havePrevVel = true;
    }

    std::size_t velCount = n - 1;
    stats.maxSpeed /= 1000.0f;
    stats.rmsSpeed = std::sqrt(speedSumSq / (float)velCount) / 1000.0f;
    stats.maxAccel /= 1000000.0f;
    if (!accelValues.empty()) {
        stats.rmsAccel = std::sqrt(accelSumSq / (float)accelValues.size()) / 1000000.0f;
        std::nth_element(accelValues.begin(),
                         accelValues.begin() + (long)(accelValues.size() / 2),
                         accelValues.end());
        stats.medianAccel = accelValues[accelValues.size() / 2] / 1000000.0f;
    }

    return stats;
}

// ---------------------------------------------------------------------------
// FFT spectrum analysis for scanner frequency content
// ---------------------------------------------------------------------------

static std::size_t nextPow2(std::size_t n) {
    std::size_t p = 1;
    while (p < n) p <<= 1;
    return p;
}

// In-place radix-2 Cooley-Tukey FFT (decimation in time)
static void fftInPlace(float* re, float* im, std::size_t N) {
    // Bit-reversal permutation
    for (std::size_t i = 1, j = 0; i < N; ++i) {
        std::size_t bit = N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            std::swap(re[i], re[j]);
            std::swap(im[i], im[j]);
        }
    }
    // Butterfly stages
    for (std::size_t len = 2; len <= N; len <<= 1) {
        float angle = -TAU / (float)len;
        float wRe = std::cos(angle), wIm = std::sin(angle);
        for (std::size_t i = 0; i < N; i += len) {
            float curRe = 1.0f, curIm = 0.0f;
            for (std::size_t j = 0; j < len / 2; ++j) {
                std::size_t u = i + j, v = i + j + len / 2;
                float tRe = curRe * re[v] - curIm * im[v];
                float tIm = curRe * im[v] + curIm * re[v];
                re[v] = re[u] - tRe;
                im[v] = im[u] - tIm;
                re[u] += tRe;
                im[u] += tIm;
                float tmp = curRe * wRe - curIm * wIm;
                curIm = curRe * wIm + curIm * wRe;
                curRe = tmp;
            }
        }
    }
}

// Compute position frequency spectrum from simulated scanner positions.
static SpectrumBands calculateSpectrum(const SimResult& sim, float pointRate) {
    SpectrumBands bands;
    if (sim.x.size() < 8 || pointRate <= 0) return bands;

    // Tile the periodic simulated positions to fill a power-of-2 FFT buffer.
    constexpr std::size_t minSamples = 4096;
    std::size_t frameLen = sim.x.size();
    std::size_t N = nextPow2(std::max(frameLen, minSamples));

    std::vector<float> xRe(N, 0.0f), xIm(N, 0.0f);
    std::vector<float> yRe(N, 0.0f), yIm(N, 0.0f);

    for (std::size_t i = 0; i < N; ++i) {
        xRe[i] = sim.x[i % frameLen];
        yRe[i] = sim.y[i % frameLen];
    }

    fftInPlace(xRe.data(), xIm.data(), N);
    fftInPlace(yRe.data(), yIm.data(), N);

    // Combined magnitude per bin (positive frequencies only), normalised by N
    std::size_t numBins = N / 2;
    std::vector<float> mag(numBins);
    float freqRes = pointRate / (float)N;
    float invN = 1.0f / (float)N;
    for (std::size_t i = 1; i < numBins; ++i) {
        float mx = std::sqrt(xRe[i] * xRe[i] + xIm[i] * xIm[i]) * invN;
        float my = std::sqrt(yRe[i] * yRe[i] + yIm[i] * yIm[i]) * invN;
        mag[i] = mx + my; // position spectrum (degrees)
    }

    // Fixed logarithmic bands from 20 Hz to 25 kHz
    constexpr float logMin = 4.321928f;  // log2(20)
    constexpr float logMax = 14.609640f; // log2(25000)

    for (int b = 0; b < EQ_NUM_BANDS; ++b) {
        float lo = std::pow(2.0f, logMin + (logMax - logMin) * (float)b / EQ_NUM_BANDS);
        float hi = std::pow(2.0f, logMin + (logMax - logMin) * (float)(b + 1) / EQ_NUM_BANDS);
        bands.freqCenter[b] = std::sqrt(lo * hi); // geometric mean

        int binLo = std::max(1, (int)(lo / freqRes));
        int binHi = std::min((int)numBins - 1, (int)(hi / freqRes));

        float peak = 0.0f;
        if (binLo <= binHi) {
            for (int i = binLo; i <= binHi; ++i) {
                if (mag[i] > peak) peak = mag[i];
            }
        }
        bands.magnitude[b] = peak;
        if (peak > bands.maxMagnitude) bands.maxMagnitude = peak;
    }

    return bands;
}

// ---------------------------------------------------------------------------
// Preview drawing with interactive output rect
// ---------------------------------------------------------------------------

static void drawPreview(AppState& state, ImVec2 pos, ImVec2 size) {
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    drawList->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), IM_COL32(0, 0, 0, 255));
    drawList->AddRect(pos, ImVec2(pos.x + size.x, pos.y + size.y), IM_COL32(80, 80, 80, 255));

    float previewSize = state.outputSize / 100.0f;
    float previewCx = state.outputX / 100.0f;
    float previewCy = state.outputY / 100.0f;

    // Mapping from normalised (-1..1) coords to screen pixels.
    // In zoomed mode, the output area fills the preview with 10% padding.
    float viewScale, viewOffX, viewOffY;
    if (state.previewZoomed) {
        float s = previewSize * 0.8f;
        float pad = 1.1f; // 10% padding around the output area
        viewScale = 1.0f / (s * pad);
        viewOffX = previewCx;
        viewOffY = previewCy;
    } else {
        viewScale = 1.0f;
        viewOffX = 0.0f;
        viewOffY = 0.0f;
    }
    auto mapX = [&](float x) { return pos.x + ((x - viewOffX) * viewScale + 1.0f) * 0.5f * size.x; };
    auto mapY = [&](float y) { return pos.y + (-(y - viewOffY) * viewScale + 1.0f) * 0.5f * size.y; };

    // Clip drawing to preview area
    drawList->PushClipRect(pos, ImVec2(pos.x + size.x, pos.y + size.y), true);

    float previewBrightness = state.armed ? 1.0f : 0.47f;
    bool showSource = (state.simOverlay == 0 || state.simOverlay == 1);
    bool showSim    = (state.simOverlay == 1 || state.simOverlay == 2);

    // Source pattern
    if (showSource) {
        if (state.outputMode == 0) {
            // Draw from previewFrame (single colour applied, before brightness)
            float brightness = state.armed ? 1.0f : 0.47f;
            for (std::size_t i = 1; i < state.previewFrame.points.size(); ++i) {
                auto& p0 = state.previewFrame.points[i - 1];
                auto& p1 = state.previewFrame.points[i];
                uint8_t cr = static_cast<uint8_t>(std::min(255.0f, p1.r * 255.0f * brightness));
                uint8_t cg = static_cast<uint8_t>(std::min(255.0f, p1.g * 255.0f * brightness));
                uint8_t cb = static_cast<uint8_t>(std::min(255.0f, p1.b * 255.0f * brightness));
                if (cr == 0 && cg == 0 && cb == 0) continue;
                drawList->AddLine(
                    ImVec2(mapX(p0.x), mapY(p0.y)),
                    ImVec2(mapX(p1.x), mapY(p1.y)),
                    IM_COL32(cr, cg, cb, 255), 1.5f);
            }
        } else if (state.outputMode == 1) {
            auto positions = getPointPositions(state.pointPatternIndex, previewCx, previewCy, previewSize);
            for (auto& dot : positions) {
                float dotX = mapX(dot.x);
                float dotY = mapY(dot.y);
                uint8_t cr = static_cast<uint8_t>(dot.r * previewBrightness * 255);
                uint8_t cg = static_cast<uint8_t>(dot.g * previewBrightness * 255);
                uint8_t cb = static_cast<uint8_t>(dot.b * previewBrightness * 255);
                drawList->AddCircleFilled(ImVec2(dotX, dotY), 4.0f, IM_COL32(cr, cg, cb, 255));
                drawList->AddLine(ImVec2(dotX - 8, dotY), ImVec2(dotX + 8, dotY), IM_COL32(80, 80, 80, 140));
                drawList->AddLine(ImVec2(dotX, dotY - 8), ImVec2(dotX, dotY + 8), IM_COL32(80, 80, 80, 140));
            }
        } else if (state.outputMode == 2) {
            // Draw from previewFrame (single colour applied, before brightness)
            float brightness = state.armed ? 1.0f : 0.47f;
            for (std::size_t i = 1; i < state.previewFrame.points.size(); ++i) {
                auto& p0 = state.previewFrame.points[i - 1];
                auto& p1 = state.previewFrame.points[i];
                uint8_t cr = static_cast<uint8_t>(std::min(255.0f, p1.r * 255.0f * brightness));
                uint8_t cg = static_cast<uint8_t>(std::min(255.0f, p1.g * 255.0f * brightness));
                uint8_t cb = static_cast<uint8_t>(std::min(255.0f, p1.b * 255.0f * brightness));
                if (cr == 0 && cg == 0 && cb == 0) continue;
                drawList->AddLine(
                    ImVec2(mapX(p0.x), mapY(p0.y)),
                    ImVec2(mapX(p1.x), mapY(p1.y)),
                    IM_COL32(cr, cg, cb, 255), 1.5f);
            }
        }
    }

    // Simulated scanner path
    if (showSim && state.simResult.x.size() >= 2) {
        constexpr float halfAngle = MAX_SCAN_ANGLE * 0.5f;
        auto simMapX = [&](float deg) { return mapX(deg / halfAngle); };
        auto simMapY = [&](float deg) { return mapY(deg / halfAngle); };
        // Brighter when sim-only, dimmer when overlaid on source
        ImU32 simCol = (state.simOverlay == 2)
            ? IM_COL32(0, 160, 255, 200)
            : IM_COL32(0, 160, 255, 100);
        for (std::size_t i = 1; i < state.simResult.x.size(); ++i) {
            drawList->AddLine(
                ImVec2(simMapX(state.simResult.x[i - 1]), simMapY(state.simResult.y[i - 1])),
                ImVec2(simMapX(state.simResult.x[i]),     simMapY(state.simResult.y[i])),
                simCol, 1.0f);
        }
    }

    if (!state.armed) {
        const char* text = "DISARMED";
        ImVec2 textSize = ImGui::CalcTextSize(text);
        drawList->AddText(
            ImVec2(pos.x + (size.x - textSize.x) * 0.5f, pos.y + size.y - textSize.y - 8.0f),
            IM_COL32(100, 100, 100, 255), text);
    }

    drawList->PopClipRect();

    // Output bounding box
    float s = previewSize * 0.8f;

    float rectLeft   = mapX(previewCx - s);
    float rectTop    = mapY(previewCy + s);
    float rectRight  = mapX(previewCx + s);
    float rectBottom = mapY(previewCy - s);
    if (rectTop > rectBottom) std::swap(rectTop, rectBottom);
    if (rectLeft > rectRight) std::swap(rectLeft, rectRight);

    // Overlay buttons (rendered before InvisibleButton so they get click priority)
    if (state.showAdvanced) {
        ImVec2 btnSize(24, 24);
        float btnY = pos.y + 4.0f;
        ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(40, 40, 40, 180));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(70, 70, 70, 220));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(100, 100, 100, 255));

        const char* simIcons[] = { ICON_FK_PENCIL, ICON_FK_EXCHANGE, ICON_FK_LINE_CHART };
        const char* simTooltips[] = { "Source only", "Source + Sim", "Sim only" };
        float btnX = pos.x + size.x - btnSize.x * 2 - 8.0f;
        ImGui::SetCursorScreenPos(ImVec2(btnX, btnY));
        if (ImGui::Button(simIcons[state.simOverlay], btnSize))
            state.simOverlay = (state.simOverlay + 1) % 3;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", simTooltips[state.simOverlay]);

        const char* zoomIcon = state.previewZoomed ? ICON_FK_SEARCH_MINUS : ICON_FK_SEARCH_PLUS;
        btnX = pos.x + size.x - btnSize.x - 4.0f;
        ImGui::SetCursorScreenPos(ImVec2(btnX, btnY));
        if (ImGui::Button(zoomIcon, btnSize))
            state.previewZoomed = !state.previewZoomed;

        ImGui::PopStyleColor(3);
    }

    ImGui::SetCursorScreenPos(pos);
    ImGui::InvisibleButton("##preview", size);

    bool doubleClicked = ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);

    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mouse = io.MousePos;

    float margin = 20.0f;
    bool nearRect = (mouse.x >= rectLeft - margin && mouse.x <= rectRight + margin &&
                     mouse.y >= rectTop - margin && mouse.y <= rectBottom + margin);
    bool hovered = ImGui::IsItemHovered() && nearRect;
    bool active = state.draggingOutput || state.resizingOutput;

    float handleSize = 6.0f;  // visual size
    float hitSize = 10.0f;   // click/hover area — generous to avoid missing
    struct Corner { float x, y; int cursor; };
    Corner corners[4] = {
        {rectLeft,  rectTop,    ImGuiMouseCursor_ResizeNWSE},
        {rectRight, rectTop,    ImGuiMouseCursor_ResizeNESW},
        {rectRight, rectBottom, ImGuiMouseCursor_ResizeNWSE},
        {rectLeft,  rectBottom, ImGuiMouseCursor_ResizeNESW},
    };

    int hoveredCorner = -1;
    bool overRect = (mouse.x >= rectLeft && mouse.x <= rectRight &&
                     mouse.y >= rectTop && mouse.y <= rectBottom);

    for (int i = 0; i < 4; ++i) {
        if (std::abs(mouse.x - corners[i].x) <= hitSize &&
            std::abs(mouse.y - corners[i].y) <= hitSize) {
            hoveredCorner = i;
            break;
        }
    }

    if (hovered || active) {
        drawList->AddRect(ImVec2(rectLeft, rectTop), ImVec2(rectRight, rectBottom),
                          IM_COL32(66, 150, 250, 180), 0.0f, 0, 1.0f);
        for (int i = 0; i < 4; ++i) {
            ImU32 handleCol = (hoveredCorner == i || state.resizingCorner == i)
                ? IM_COL32(100, 180, 255, 255) : IM_COL32(66, 150, 250, 200);
            drawList->AddRectFilled(
                ImVec2(corners[i].x - handleSize, corners[i].y - handleSize),
                ImVec2(corners[i].x + handleSize, corners[i].y + handleSize), handleCol);
        }
    }

    if (ImGui::IsItemHovered()) {
        if (hoveredCorner >= 0)
            ImGui::SetMouseCursor(static_cast<ImGuiMouseCursor>(corners[hoveredCorner].cursor));
        else if (overRect)
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
    }

    if (doubleClicked && overRect)
        state.resetOutput();

    if (ImGui::IsItemActive()) {
        // On initial click, capture what we're interacting with
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            if (hoveredCorner >= 0) {
                state.resizingOutput = true;
                state.resizingCorner = hoveredCorner;
            } else if (overRect) {
                state.draggingOutput = true;
            }
        }

        // Apply drag
        if (state.resizingOutput) {
            float centerScreenX = (rectLeft + rectRight) * 0.5f;
            float centerScreenY = (rectTop + rectBottom) * 0.5f;
            float dist = std::max(std::abs(mouse.x - centerScreenX),
                                  std::abs(mouse.y - centerScreenY));
            float newSize = dist / (0.8f * 0.5f * size.x) * 100.0f;
            state.outputSize = std::clamp(newSize, 0.0f, 100.0f);
        } else if (state.draggingOutput) {
            float dx = io.MouseDelta.x / size.x * 2.0f * 100.0f;
            float dy = io.MouseDelta.y / size.y * 2.0f * 100.0f;
            state.outputX = std::clamp(state.outputX + dx, -100.0f, 100.0f);
            state.outputY = std::clamp(state.outputY - dy, -100.0f, 100.0f);
        }
    } else {
        state.draggingOutput = false;
        state.resizingOutput = false;
        state.resizingCorner = -1;
    }
}

// ---------------------------------------------------------------------------
// Background discovery thread
// ---------------------------------------------------------------------------

static void discoveryThreadFunc(AppState& state) {
    while (state.discoveryRunning.load()) {
        auto discovered = state.liberaSystem.discoverControllers();
        {
            std::lock_guard<std::mutex> lock(state.discoveredMutex);
            state.latestDiscovered.clear();
            for (auto& d : discovered)
                state.latestDiscovered.push_back({d->idValue(), d->labelValue(), d->type(), d->maxPointRate(), d->usageState()});
            state.discoveryResultReady.store(true);
        }
        {
            std::unique_lock<std::mutex> lk(state.discoveryCvMutex);
            state.discoveryCv.wait_for(lk, 2000ms, [&] {
                return !state.discoveryRunning.load() || state.discoveryRequested.load();
            });
            state.discoveryRequested.store(false);
        }
    }
    state.discoveryFinished.store(true, std::memory_order_release);
}

static void startAsyncConnect(AppState& state, ControllerEntry& entry);

static void applyDiscoveryResults(AppState& state) {
    if (!state.discoveryResultReady.load()) return;
    std::vector<DiscoveredInfo> discovered;
    {
        std::lock_guard<std::mutex> lock(state.discoveredMutex);
        discovered = std::move(state.latestDiscovered);
        state.discoveryResultReady.store(false);
    }
    for (auto& entry : state.controllers) {
        bool stillPresent = false;
        for (auto& d : discovered) {
            if (d.id == entry.id) {
                stillPresent = true;
                entry.usageState = d.usageState;
                break;
            }
        }
        if (!stillPresent && entry.controller) { entry.controller.reset(); entry.enabled = false; }
    }
    for (auto& d : discovered) {
        bool exists = false;
        for (auto& entry : state.controllers) { if (entry.id == d.id) { exists = true; break; } }
        if (!exists) {
            bool shouldAutoConnect = state.savedEnabledControllers.count(d.id) > 0;
            state.controllers.push_back({d.id, d.label, d.type, d.maxPointRate, d.usageState, shouldAutoConnect, false, nullptr, {}});
            if (shouldAutoConnect) startAsyncConnect(state, state.controllers.back());
        }
    }
}

static void pollAsyncConnections(AppState& state) {
    for (auto& entry : state.controllers) {
        if (entry.connecting && entry.connectFuture.valid()) {
            if (entry.connectFuture.wait_for(0ms) == std::future_status::ready) {
                entry.controller = entry.connectFuture.get();
                entry.connecting = false;
                if (entry.controller)
                    entry.controller->setPointRate(static_cast<std::uint32_t>(state.effectivePointRate()));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Frame building
// ---------------------------------------------------------------------------

static core::Frame buildCurrentFrame(AppState& state) {
    float sz = state.outputSize / 100.0f;
    float cx = state.outputX / 100.0f;
    float cy = state.outputY / 100.0f;

    core::Frame frame;
    if (state.outputMode == 1) {
        frame = makePointFrame(state.pointPatternIndex, cx, cy, sz, state.dutyCycle);
    } else if (state.outputMode == 2) {
        int idx = state.selectedIldaPattern;
        if (idx >= 0 && idx < static_cast<int>(state.ildaPatterns.size())) {
            const auto& pat = state.ildaPatterns[idx];
            int frameCount = static_cast<int>(pat.frames.size());
            if (frameCount > 1 && state.ildaPlaying && pat.fps > 0.0f) {
                // Advance frame index based on elapsed wall-clock time
                auto now = std::chrono::steady_clock::now();
                float elapsed = std::chrono::duration<float>(
                    now - state.ildaLastAdvance).count();
                float frameDuration = 1.0f / pat.fps;
                if (elapsed >= frameDuration) {
                    int steps = static_cast<int>(elapsed / frameDuration);
                    int next = state.ildaFrameIndex + steps;
                    if (state.ildaLoop) {
                        next %= frameCount;
                    } else if (next >= frameCount) {
                        next = frameCount - 1;
                        state.ildaPlaying = false;
                    }
                    state.ildaFrameIndex = next;
                    state.ildaLastAdvance = now;
                }
            }
            if (state.ildaFrameIndex >= frameCount) state.ildaFrameIndex = 0;
            if (state.ildaFrameIndex < 0) state.ildaFrameIndex = 0;
            frame = makeILDAFrame(pat.frames[state.ildaFrameIndex], sz * 0.8f, cx, cy);
        } else {
            frame = makeShapeFrame(0, sz, cx, cy); // fallback
        }
    } else {
        frame = makeShapeFrame(state.patternIndex, sz, cx, cy);
    }

    // Single colour: convert to white using max channel intensity
    if (state.singleColour) {
        for (auto& pt : frame.points) {
            float m = std::max({pt.r, pt.g, pt.b});
            pt.r = m;
            pt.g = m;
            pt.b = m;
        }
    }

    // Apply RGB channel enables and levels (without main brightness)
    float rScale = state.redEnabled   ? (state.red   / 100.0f) : 0.0f;
    float gScale = state.greenEnabled ? (state.green / 100.0f) : 0.0f;
    float bScale = state.blueEnabled  ? (state.blue  / 100.0f) : 0.0f;
    for (auto& pt : frame.points) {
        pt.r *= rScale;
        pt.g *= gScale;
        pt.b *= bScale;
    }

    // Snapshot for preview (after colour, before brightness)
    state.previewFrame = frame;

    // Apply main brightness
    float br = state.brightness / 100.0f;
    for (auto& pt : frame.points) {
        pt.r *= br;
        pt.g *= br;
        pt.b *= br;
    }

    return frame;
}

// ---------------------------------------------------------------------------
// Frame sending
// ---------------------------------------------------------------------------

// Build the current frame and compute scan stats + spectrum once per loop.
// Called before UI rendering so all gauges read from state.scanStats / state.spectrum.
static void updateFrameAndStats(AppState& state) {
    state.currentFrame = buildCurrentFrame(state);

    float pps = static_cast<float>(state.effectivePointRate());
    if (state.currentFrame.points.size() >= 3) {
        state.simResult = simulateScanner(state.currentFrame, pps, scannerSim);
        int stepsPerPoint = std::max(1, (int)(scannerSim.simRate / pps + 0.5f));
        float simPps = pps * (float)stepsPerPoint;
        state.scanStats = calculateScanStats(state.simResult, simPps);
        state.spectrum = calculateSpectrum(state.simResult, simPps);
    } else {
        state.simResult = {};
        state.scanStats = {};
        state.spectrum = {};
    }
}

static void sendFramesToControllers(AppState& state) {
    core::Frame frame = state.currentFrame;

    for (auto& pt : frame.points)
        applyTransform(pt.x, pt.y, state);

    for (auto& entry : state.controllers) {
        if (!entry.enabled || !entry.controller) continue;
        entry.controller->setArmed(state.armed);
        entry.controller->setPointRate(static_cast<std::uint32_t>(state.effectivePointRate()));
        entry.controller->setScannerSync(static_cast<double>(state.scannerSync));
        if (entry.controller->isReadyForNewFrame()) {
            core::Frame copy = frame;
            entry.controller->sendFrame(std::move(copy));
        }
    }
}

static void startAsyncConnect(AppState& state, ControllerEntry& entry) {
    entry.connecting = true;
    std::string entryId = entry.id;
    System* sys = &state.liberaSystem;
    entry.connectFuture = std::async(std::launch::async, [sys, entryId]() -> std::shared_ptr<core::LaserController> {
        auto discovered = sys->discoverControllers();
        for (auto& d : discovered) {
            if (d->idValue() == entryId) return sys->connectController(*d);
        }
        return nullptr;
    });
}

static void disconnectController(ControllerEntry& entry) {
    if (entry.controller) { entry.controller->setArmed(false); entry.controller.reset(); }
    entry.connecting = false;
}

// ---------------------------------------------------------------------------
// Custom toggle button widget — sized to match slider height
// ---------------------------------------------------------------------------

static bool toggleButton(const char* label, bool* v, bool showConnecting = false, float sizeOverride = 0.0f, const char* text = nullptr) {
    ImGui::PushID(label);
    bool clicked = false;
    bool on = *v;

    float frameH = ImGui::GetFrameHeight();
    float sz = (sizeOverride > 0) ? sizeOverride : frameH;
    float yPad = (frameH - sz) * 0.5f;

    // Calculate total width including text label if provided
    float totalW = sz;
    if (text) {
        float textW = ImGui::CalcTextSize(text).x;
        totalW += ImGui::GetStyle().ItemInnerSpacing.x + textW;
    }

    ImVec2 cursor = ImGui::GetCursorScreenPos();
    ImVec2 p(cursor.x, cursor.y + yPad);
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    if (ImGui::InvisibleButton("##toggle", ImVec2(totalW, frameH))) {
        *v = !*v;
        clicked = true;
        on = *v;
    }
    bool hovered = ImGui::IsItemHovered();

    ImU32 outerCol = hovered ? IM_COL32(66, 150, 250, 255) : IM_COL32(66, 150, 250, 102);
    drawList->AddRectFilled(p, ImVec2(p.x + sz, p.y + sz), outerCol, 2.0f);

    float m = sz * 0.3f; // ~30% margin each side, inner is ~40% of outer
    bool showInner = false;
    if (showConnecting) {
        double t = ImGui::GetTime();
        showInner = (static_cast<int>(t * 4.0) % 2 == 0);
    } else if (on) {
        showInner = true;
    }
    if (showInner) {
        drawList->AddRectFilled(
            ImVec2(p.x + m, p.y + m), ImVec2(p.x + sz - m, p.y + sz - m),
            IM_COL32(255, 255, 255, 255), 1.0f);
    }

    // Draw text label if provided
    if (text) {
        float textX = p.x + sz + ImGui::GetStyle().ItemInnerSpacing.x;
        float textY = cursor.y + (frameH - ImGui::GetTextLineHeight()) * 0.5f;
        ImU32 textCol = hovered ? IM_COL32(255, 255, 255, 255) : IM_COL32(200, 200, 200, 255);
        drawList->AddText(ImVec2(textX, textY), textCol, text);
    }

    ImGui::PopID();
    return clicked;
}

// ---------------------------------------------------------------------------
// Draw the LIBERA LAB logo
// ---------------------------------------------------------------------------

static void openURL(const char* url) {
#ifdef __APPLE__
    std::string cmd = std::string("open ") + url;
#elif defined(_WIN32)
    std::string cmd = std::string("start ") + url;
#else
    std::string cmd = std::string("xdg-open ") + url;
#endif
    std::system(cmd.c_str());
}

// Returns true if the logo was drawn (fits in the area).
// Draws logo text + subtitle, and handles click to open URL.
static bool drawLogo(ImGuiIO& io, ImVec2 pos, float areaWidth,
                     bool rightAlign = true, ImDrawList* overrideDl = nullptr) {
    ImDrawList* drawList = overrideDl ? overrideDl : ImGui::GetWindowDrawList();
    ImFont* boldFont = io.Fonts->Fonts[2];
    ImFont* defaultFont = io.Fonts->Fonts[0];
    float fontScale = ImGui::GetStyle().FontScaleMain;
    float logoFontSize = boldFont->LegacySize * fontScale;
    float subFontSize = defaultFont->LegacySize * fontScale * 0.85f;

    const char* t1 = "LIBERA";
    const char* t2 = "LAB";
    const char* sub = "A LIBERATION PROJECT";
    ImVec2 s1 = boldFont->CalcTextSizeA(logoFontSize, FLT_MAX, 0, t1);
    ImVec2 s2 = boldFont->CalcTextSizeA(logoFontSize, FLT_MAX, 0, t2);
    ImVec2 subSize = defaultFont->CalcTextSizeA(subFontSize, FLT_MAX, 0, sub);
    float logoW = s1.x + 4.0f + s2.x;
    float totalW = std::max(logoW, subSize.x);
    if (totalW > areaWidth) return false;

    float x = rightAlign ? pos.x + areaWidth - totalW : pos.x;
    float y = pos.y;

    // Logo text
    float logoX = rightAlign ? pos.x + areaWidth - logoW : pos.x;
    drawList->AddText(boldFont, logoFontSize, ImVec2(logoX, y),
                      IM_COL32(220, 50, 220, 255), t1);
    drawList->AddText(boldFont, logoFontSize, ImVec2(logoX + s1.x + 4.0f, y),
                      IM_COL32(153, 153, 153, 255), t2);

    // Subtitle
    float subX = rightAlign ? pos.x + areaWidth - subSize.x : pos.x;
    float subY = y + s1.y + 1.0f;
    drawList->AddText(defaultFont, subFontSize, ImVec2(subX, subY),
                      IM_COL32(100, 100, 100, 200), sub);

    // Clickable area over the whole logo + subtitle (manual hit test
    // so it works even when drawn over other widgets like the preview)
    float totalH = s1.y + 1.0f + subSize.y;
    ImVec2 mouse = ImGui::GetMousePos();
    if (mouse.x >= x && mouse.x <= x + totalW &&
        mouse.y >= y && mouse.y <= y + totalH) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            openURL("https://liberationlaser.com");
    }

    return true;
}


// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int /*argc*/, char* argv[]) {
    // Register user plugin directory (alongside the default bundled "plugins"
    // next to the executable) before AppState constructs its System member.
    {
        std::string userPluginDir = getSettingsDir() + "/plugins";
#ifdef _WIN32
        CreateDirectoryA(userPluginDir.c_str(), nullptr);
#else
        mkdir(userPluginDir.c_str(), 0755);
#endif
        libera::System::addPluginDirectory(userPluginDir);
    }

    // Load settings early so we can restore window geometry
    AppState state;
    loadSettings(state);

    LiberaApp app;
    if (!app.init({"Libera Lab v" APP_VERSION, state.windowWidth, state.windowHeight,
                    state.windowX, state.windowY}))
        return 1;

    // Load ILDA patterns from patterns/ folder next to executable
    loadILDAPatterns(state, getPatternsDir(argv[0]));

    state.discoveryRunning.store(true);
    state.discoveryThread = std::thread(discoveryThreadFunc, std::ref(state));

    while (app.beginFrame()) {
        applyDiscoveryResults(state);
        pollAsyncConnections(state);
        updateFrameAndStats(state);
        sendFramesToControllers(state);

        ImGuiIO& io = ImGui::GetIO();
        ImVec4* colors = ImGui::GetStyle().Colors;

        // ---- Colour solo/toggle helper (used by keys and buttons) ----
        auto soloOrToggle = [&](bool* channel) {
            if (io.KeyCtrl) {
                bool* channels[] = { &state.redEnabled, &state.greenEnabled, &state.blueEnabled };
                bool isSolo = *channel;
                for (auto* ch : channels)
                    isSolo = isSolo && (ch == channel || !*ch);
                if (isSolo) {
                    for (auto* ch : channels) *ch = true;
                } else {
                    for (auto* ch : channels) *ch = (ch == channel);
                }
            } else {
                *channel = !*channel;
            }
        };

        // ---- Keyboard shortcuts ----
        if (!io.WantTextInput) {
            // ESC — disarm
            if (ImGui::IsKeyPressed(ImGuiKey_Escape))
                state.armed = false;

            // CMD+A (macOS) / CTRL+A (Windows/Linux) — toggle arm
            if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_A))
                state.armed = !state.armed;

            // R / G / B — toggle colour channels; CMD+R/G/B solo/restore
            if (ImGui::IsKeyPressed(ImGuiKey_R))
                soloOrToggle(&state.redEnabled);
            if (ImGui::IsKeyPressed(ImGuiKey_G))
                soloOrToggle(&state.greenEnabled);
            if (ImGui::IsKeyPressed(ImGuiKey_B))
                soloOrToggle(&state.blueEnabled);

            // 1–0 = fixed brightness levels
            {
                constexpr float levels[] = { 100.0f, 0.5f, 1.0f, 5.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 80.0f };
                for (int n = 0; n <= 9; ++n) {
                    if (ImGui::IsKeyPressed((ImGuiKey)(ImGuiKey_0 + n)))
                        state.brightness = levels[n];
                }
            }

            // =/- = brightness up/down by 1%, min 0.5%
            if (ImGui::IsKeyPressed(ImGuiKey_Equal, false))
                state.brightness = std::min(std::floor(state.brightness) + 1.0f, 100.0f);
            if (ImGui::IsKeyPressed(ImGuiKey_Minus, false))
                state.brightness = std::max(std::ceil(state.brightness) - 1.0f, 0.5f);
        }

        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::Begin("Libera Lab", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);

        float windowWidth = ImGui::GetContentRegionAvail().x;
        float windowHeight = ImGui::GetContentRegionAvail().y;
        float controlPanelWidth = 340.0f;
        float bottomPanelHeight = 180.0f;

        float previewSize = std::min(windowWidth - controlPanelWidth - 20.0f,
                                      windowHeight - bottomPanelHeight - 20.0f);
        if (previewSize < 100.0f) previewSize = 100.0f;

        ImVec2 previewPos = ImGui::GetCursorScreenPos();
        drawPreview(state, previewPos, ImVec2(previewSize, previewSize));

        // ---- Right: Controls ----
        float rightX = previewPos.x + previewSize + 16.0f;
        ImGui::SetCursorScreenPos(ImVec2(rightX, previewPos.y));
        ImGui::BeginGroup();

        float sliderWidth = controlPanelWidth - 16.0f;
        ImGui::PushItemWidth(sliderWidth);

        // ARM button
        {
            constexpr float hue = 0.98f;
            if (state.armed) {
                ImGui::PushStyleColor(ImGuiCol_Button,       (ImVec4)ImColor::HSV(hue, 0.6f, 0.6f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(hue, 0.6f, 0.9f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (ImVec4)ImColor::HSV(hue, 1.0f, 1.0f));
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button,       colors[ImGuiCol_Button]);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colors[ImGuiCol_ButtonHovered]);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive,  colors[ImGuiCol_ButtonActive]);
            }
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 6.0f);
            ImGui::PushFont(io.Fonts->Fonts[1]);
            const char* armLabel = state.armed ? ICON_FK_EXCLAMATION_TRIANGLE " ARMED" : "ARM";
            if (ImGui::Button(armLabel, ImVec2(controlPanelWidth - 16.0f, 44.0f)))
                state.armed = !state.armed;
            ImGui::PopFont();
            ImGui::PopStyleVar();
            ImGui::PopStyleColor(3);
        }

        // RGB enable buttons — under ARM
        {
            float totalW = controlPanelWidth - 16.0f;
            float spacing = 4.0f;
            float btnW = (totalW - spacing * 2.0f) / 3.0f;
            float btnH = 28.0f;

            struct RGBBtn { const char* label; float hue; bool* enabled; };
            RGBBtn btns[] = {
                {"RED",   0.0f,        &state.redEnabled},
                {"GREEN", 0.33f,       &state.greenEnabled},
                {"BLUE",  0.6f,        &state.blueEnabled},
            };

            for (int i = 0; i < 3; ++i) {
                float h = btns[i].hue;
                bool on = *btns[i].enabled;
                if (on) {
                    ImGui::PushStyleColor(ImGuiCol_Button,       (ImVec4)ImColor::HSV(h, 0.6f, 0.6f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(h, 0.6f, 0.9f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (ImVec4)ImColor::HSV(h, 1.0f, 1.0f));
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Button,       (ImVec4)ImColor::HSV(h, 0.15f, 0.25f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(h, 0.3f, 0.4f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (ImVec4)ImColor::HSV(h, 0.6f, 0.6f));
                }
                if (ImGui::Button(btns[i].label, ImVec2(btnW, btnH)))
                    soloOrToggle(btns[i].enabled);
                ImGui::PopStyleColor(3);
                if (i < 2) ImGui::SameLine(0, spacing);
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Brightness slider (no label)
        ImGui::PushItemWidth(controlPanelWidth - 16.0f);
        ImGui::SliderFloat("##brightness", &state.brightness, 0.5f, 100.0f, "%.1f%%");
        ImGui::PopItemWidth();

        // RGB sliders — three side by side, under brightness
        {
            float totalW = controlPanelWidth - 16.0f;
            float spacing = 4.0f;
            float sliderW = (totalW - spacing * 2.0f) / 3.0f;

            ImGui::PushItemWidth(sliderW);

            ImGui::PushStyleColor(ImGuiCol_SliderGrab, IM_COL32(220, 50, 50, 255));
            ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, IM_COL32(255, 80, 80, 255));
            if (!state.redEnabled) ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.3f);
            ImGui::SliderFloat("##r_ch", &state.red, 0.0f, 100.0f, "%.0f%%");
            if (!state.redEnabled) ImGui::PopStyleVar();
            ImGui::PopStyleColor(2);

            ImGui::SameLine(0, spacing);

            ImGui::PushStyleColor(ImGuiCol_SliderGrab, IM_COL32(50, 200, 50, 255));
            ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, IM_COL32(80, 255, 80, 255));
            if (!state.greenEnabled) ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.3f);
            ImGui::SliderFloat("##g_ch", &state.green, 0.0f, 100.0f, "%.0f%%");
            if (!state.greenEnabled) ImGui::PopStyleVar();
            ImGui::PopStyleColor(2);

            ImGui::SameLine(0, spacing);

            ImGui::PushStyleColor(ImGuiCol_SliderGrab, IM_COL32(50, 80, 220, 255));
            ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, IM_COL32(80, 120, 255, 255));
            if (!state.blueEnabled) ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.3f);
            ImGui::SliderFloat("##b_ch", &state.blue, 0.0f, 100.0f, "%.0f%%");
            if (!state.blueEnabled) ImGui::PopStyleVar();
            ImGui::PopStyleColor(2);

            ImGui::PopItemWidth();
        }

        ImGui::PushItemWidth(sliderWidth);

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Mode selector: SHAPE / BEAM / PATTERN
        {
            const char* modeLabels[] = {"SHAPE", "POINT", "PATTERN"};
            float btnWidth = 80.0f;
            float btnSpacing = 4.0f;
            for (int i = 0; i < 3; ++i) {
                bool selected = (state.outputMode == i);
                if (selected) {
                    ImGui::PushStyleColor(ImGuiCol_Button,       (ImVec4)ImColor(174, 84, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(206, 115, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (ImVec4)ImColor(218, 145, 65));
                }
                if (ImGui::Button(modeLabels[i], ImVec2(btnWidth, 0)))
                    state.outputMode = i;
                if (selected) ImGui::PopStyleColor(3);
                if (i < 2) ImGui::SameLine(0, btnSpacing);
            }
        }

        ImGui::Spacing();

        // Mode-specific content
        if (state.outputMode == 0) {
            // Shape mode — pattern thumbnails
            {
                float thumbSize = 80.0f;
                float spacing = 8.0f;
                ImDrawList* drawList = ImGui::GetWindowDrawList();
                float rowStartX = ImGui::GetCursorScreenPos().x;

                for (int i = 0; i < NUM_PATTERNS; ++i) {
                    ImVec2 thumbPos = ImGui::GetCursorScreenPos();
                    bool selected = (state.patternIndex == i);
                    ImU32 borderCol = selected ? IM_COL32(66, 150, 250, 255) : IM_COL32(60, 60, 60, 255);
                    float borderThickness = selected ? 2.0f : 1.0f;

                    drawList->AddRectFilled(thumbPos, ImVec2(thumbPos.x + thumbSize, thumbPos.y + thumbSize),
                                            IM_COL32(10, 10, 10, 255));
                    drawList->AddRect(thumbPos, ImVec2(thumbPos.x + thumbSize, thumbPos.y + thumbSize),
                                      borderCol, 2.0f, 0, borderThickness);

                    float thumbBright = selected ? 1.0f : 0.6f;
                    drawPatternInRect(drawList, i, 1.0f, 0.0f, 0.0f,
                                      thumbPos, ImVec2(thumbSize, thumbSize), thumbBright);

                    ImVec2 labelSize = ImGui::CalcTextSize(patternNames[i]);
                    drawList->AddText(
                        ImVec2(thumbPos.x + (thumbSize - labelSize.x) * 0.5f, thumbPos.y + thumbSize + 2.0f),
                        selected ? IM_COL32(255, 255, 255, 255) : IM_COL32(150, 150, 150, 255),
                        patternNames[i]);

                    ImGui::SetCursorScreenPos(thumbPos);
                    char btnId[32];
                    std::snprintf(btnId, sizeof(btnId), "##thumb%d", i);
                    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(0, 0, 0, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(255, 255, 255, 30));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(255, 255, 255, 50));
                    if (ImGui::Button(btnId, ImVec2(thumbSize, thumbSize)))
                        state.patternIndex = i;
                    ImGui::PopStyleColor(3);

                    bool endOfRow = ((i + 1) % 4 == 0);
                    bool lastItem = (i == NUM_PATTERNS - 1);
                    if (!endOfRow && !lastItem) {
                        ImGui::SameLine(0, spacing);
                    } else {
                        ImGui::SetCursorScreenPos(ImVec2(
                            rowStartX,
                            thumbPos.y + thumbSize + labelSize.y + 8.0f));
                    }
                }
            }

            // Single colour toggle
            ImGui::Spacing();
            {
                bool wasOn = state.singleColour;
                if (wasOn) {
                    ImGui::PushStyleColor(ImGuiCol_Button,       (ImVec4)ImColor(174, 84, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(206, 115, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (ImVec4)ImColor(218, 145, 65));
                }
                if (ImGui::Button("SINGLE COLOUR MODE##shape"))
                    state.singleColour = !state.singleColour;
                if (wasOn) ImGui::PopStyleColor(3);
            }
        } else if (state.outputMode == 1) {
            // Point mode — pattern thumbnails
            {
                float thumbSize = 60.0f;
                float spacing = 6.0f;
                ImDrawList* ptDrawList = ImGui::GetWindowDrawList();
                float ptRowStartX = ImGui::GetCursorScreenPos().x;

                for (int i = 0; i < NUM_POINT_PATTERNS; ++i) {
                    ImVec2 thumbPos = ImGui::GetCursorScreenPos();
                    bool selected = (state.pointPatternIndex == i);
                    ImU32 borderCol = selected ? IM_COL32(66, 150, 250, 255) : IM_COL32(60, 60, 60, 255);
                    float borderThickness = selected ? 2.0f : 1.0f;

                    ptDrawList->AddRectFilled(thumbPos,
                        ImVec2(thumbPos.x + thumbSize, thumbPos.y + thumbSize),
                        IM_COL32(10, 10, 10, 255));
                    ptDrawList->AddRect(thumbPos,
                        ImVec2(thumbPos.x + thumbSize, thumbPos.y + thumbSize),
                        borderCol, 2.0f, 0, borderThickness);

                    // Draw the dot pattern in the thumbnail
                    float tb = selected ? 1.0f : 0.6f;
                    auto positions = getPointPositions(i, 0.0f, 0.0f, 1.0f);
                    for (auto& dot : positions) {
                        float dx = thumbPos.x + (dot.x + 1.0f) * 0.5f * thumbSize;
                        float dy = thumbPos.y + ((-dot.y) + 1.0f) * 0.5f * thumbSize;
                        uint8_t cr = static_cast<uint8_t>(dot.r * tb * 255);
                        uint8_t cg = static_cast<uint8_t>(dot.g * tb * 255);
                        uint8_t cb = static_cast<uint8_t>(dot.b * tb * 255);
                        ptDrawList->AddCircleFilled(ImVec2(dx, dy), 3.0f, IM_COL32(cr, cg, cb, 255));
                    }

                    // Label
                    ImVec2 labelSize = ImGui::CalcTextSize(pointPatternNames[i]);
                    ptDrawList->AddText(
                        ImVec2(thumbPos.x + (thumbSize - labelSize.x) * 0.5f,
                               thumbPos.y + thumbSize + 2.0f),
                        selected ? IM_COL32(255, 255, 255, 255) : IM_COL32(150, 150, 150, 255),
                        pointPatternNames[i]);

                    ImGui::SetCursorScreenPos(thumbPos);
                    char btnId[32];
                    std::snprintf(btnId, sizeof(btnId), "##ptpat%d", i);
                    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(0, 0, 0, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(255, 255, 255, 30));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(255, 255, 255, 50));
                    if (ImGui::Button(btnId, ImVec2(thumbSize, thumbSize)))
                        state.pointPatternIndex = i;
                    ImGui::PopStyleColor(3);

                    bool endOfRow = ((i + 1) % 5 == 0);
                    bool lastItem = (i == NUM_POINT_PATTERNS - 1);
                    if (!endOfRow && !lastItem) {
                        ImGui::SameLine(0, spacing);
                    } else {
                        ImGui::SetCursorScreenPos(ImVec2(
                            ptRowStartX,
                            thumbPos.y + thumbSize + labelSize.y + 8.0f));
                    }
                }
            }

            ImGui::Text("Duty Cycle");
            ImGui::PushItemWidth(controlPanelWidth - 16.0f);
            ImGui::SliderFloat("##duty", &state.dutyCycle, 1.0f, 100.0f, "%.0f%%");
            ImGui::PopItemWidth();

            // Single colour toggle
            ImGui::Spacing();
            {
                bool wasOn = state.singleColour;
                if (wasOn) {
                    ImGui::PushStyleColor(ImGuiCol_Button,       (ImVec4)ImColor(174, 84, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(206, 115, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (ImVec4)ImColor(218, 145, 65));
                }
                if (ImGui::Button("SINGLE COLOUR MODE##point"))
                    state.singleColour = !state.singleColour;
                if (wasOn) ImGui::PopStyleColor(3);
            }
        } else {
            // Pattern mode — ILDA pattern grid
            if (state.ildaPatterns.empty()) {
                ImGui::TextDisabled("No .ild files found in patterns/ folder");
            } else {
                float thumbSize = 80.0f;
                float spacing = 8.0f;
                ImDrawList* ildaDrawList = ImGui::GetWindowDrawList();
                int cols = 4;
                float ildaRowStartX = ImGui::GetCursorScreenPos().x;

                for (int i = 0; i < static_cast<int>(state.ildaPatterns.size()); ++i) {
                    ImVec2 thumbPos = ImGui::GetCursorScreenPos();
                    bool selected = (state.selectedIldaPattern == i);
                    ImU32 borderCol = selected ? IM_COL32(66, 150, 250, 255) : IM_COL32(60, 60, 60, 255);
                    float borderThickness = selected ? 2.0f : 1.0f;

                    ildaDrawList->AddRectFilled(thumbPos,
                        ImVec2(thumbPos.x + thumbSize, thumbPos.y + thumbSize),
                        IM_COL32(10, 10, 10, 255));
                    ildaDrawList->AddRect(thumbPos,
                        ImVec2(thumbPos.x + thumbSize, thumbPos.y + thumbSize),
                        borderCol, 2.0f, 0, borderThickness);

                    const auto& pat = state.ildaPatterns[i];
                    // For the currently selected multi-frame pattern, show the live frame;
                    // otherwise show the first frame as a static thumbnail.
                    int thumbFrameIdx = 0;
                    if (selected && pat.frames.size() > 1 &&
                        state.ildaFrameIndex >= 0 &&
                        state.ildaFrameIndex < static_cast<int>(pat.frames.size())) {
                        thumbFrameIdx = state.ildaFrameIndex;
                    }
                    drawILDAInRect(ildaDrawList, pat.frames[thumbFrameIdx],
                                   thumbPos, ImVec2(thumbSize, thumbSize), true,
                                   selected ? 1.0f : 0.6f);

                    // Animation badge: show frame count in corner if multi-frame
                    if (pat.frames.size() > 1) {
                        char badge[16];
                        std::snprintf(badge, sizeof(badge), "%zu", pat.frames.size());
                        ImVec2 badgeSize = ImGui::CalcTextSize(badge);
                        ImVec2 badgePos(thumbPos.x + thumbSize - badgeSize.x - 6.0f,
                                        thumbPos.y + 3.0f);
                        ildaDrawList->AddRectFilled(
                            ImVec2(badgePos.x - 3.0f, badgePos.y - 1.0f),
                            ImVec2(badgePos.x + badgeSize.x + 3.0f, badgePos.y + badgeSize.y + 1.0f),
                            IM_COL32(0, 0, 0, 180), 2.0f);
                        ildaDrawList->AddText(badgePos, IM_COL32(255, 200, 80, 255), badge);
                    }

                    // Label — truncate with ellipsis if it won't fit under the thumbnail
                    std::string shownName = pat.name;
                    ImVec2 labelSize = ImGui::CalcTextSize(shownName.c_str());
                    if (labelSize.x > thumbSize) {
                        const std::string ell = "...";
                        float ellW = ImGui::CalcTextSize(ell.c_str()).x;
                        while (shownName.size() > 1 &&
                               ImGui::CalcTextSize(shownName.c_str()).x + ellW > thumbSize) {
                            shownName.pop_back();
                        }
                        shownName += ell;
                        labelSize = ImGui::CalcTextSize(shownName.c_str());
                    }
                    ildaDrawList->AddText(
                        ImVec2(thumbPos.x + std::max(0.0f, (thumbSize - labelSize.x) * 0.5f),
                               thumbPos.y + thumbSize + 2.0f),
                        selected ? IM_COL32(255, 255, 255, 255) : IM_COL32(150, 150, 150, 255),
                        shownName.c_str());

                    ImGui::SetCursorScreenPos(thumbPos);
                    char btnId[32];
                    std::snprintf(btnId, sizeof(btnId), "##ilda%d", i);
                    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(0, 0, 0, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(255, 255, 255, 30));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(255, 255, 255, 50));
                    if (ImGui::Button(btnId, ImVec2(thumbSize, thumbSize))) {
                        if (state.selectedIldaPattern != i) {
                            state.selectedIldaPattern = i;
                            state.ildaFrameIndex = 0;
                            state.ildaLastAdvance = std::chrono::steady_clock::now();
                        }
                    }
                    ImGui::PopStyleColor(3);

                    int col = i % cols;
                    if (col < cols - 1 && i < static_cast<int>(state.ildaPatterns.size()) - 1) {
                        ImGui::SameLine(0, spacing);
                    } else {
                        ImGui::SetCursorScreenPos(ImVec2(
                            ildaRowStartX,
                            thumbPos.y + thumbSize + labelSize.y + 8.0f));
                    }
                }
            }

            // "Load ILDA file…" button — always available, even with no patterns loaded
            ImGui::Spacing();
            if (ImGui::Button("Load ILDA file...")) {
                std::string picked = libera::ui::OpenFileDialog("Load ILDA file", {"ild", "ILD"});
                if (!picked.empty()) {
                    int newIdx = addILDAFile(state, picked);
                    if (newIdx >= 0) {
                        state.selectedIldaPattern = newIdx;
                        state.ildaFrameIndex = 0;
                        state.ildaLastAdvance = std::chrono::steady_clock::now();
                    }
                }
            }

            // Delete button — only active when a user-loaded pattern is selected
            ImGui::SameLine();
            int selForDelete = state.selectedIldaPattern;
            bool canDelete = (selForDelete >= 0 &&
                              selForDelete < static_cast<int>(state.ildaPatterns.size()) &&
                              !state.ildaPatterns[selForDelete].builtin);
            ImGui::BeginDisabled(!canDelete);
            if (ImGui::Button("Delete") && canDelete) {
                // Only remove the file if it lives inside the user patterns dir
                std::error_code ec;
                std::filesystem::path filePath(state.ildaPatterns[selForDelete].path);
                std::filesystem::path userDir(getUserPatternsDir());
                auto rel = std::filesystem::relative(filePath, userDir, ec);
                // Reject any relative path that climbs above the user dir.
                // .native() is wstring on Windows / string on POSIX, so compare
                // path components directly instead of using string search.
                bool escapesUserDir = false;
                if (ec || rel.empty()) {
                    escapesUserDir = true;
                } else {
                    for (const auto& part : rel) {
                        if (part == std::filesystem::path("..")) {
                            escapesUserDir = true;
                            break;
                        }
                    }
                }
                if (!escapesUserDir) std::filesystem::remove(filePath, ec);
                state.ildaPatterns.erase(state.ildaPatterns.begin() + selForDelete);
                if (state.selectedIldaPattern >= static_cast<int>(state.ildaPatterns.size()))
                    state.selectedIldaPattern = static_cast<int>(state.ildaPatterns.size()) - 1;
                if (state.selectedIldaPattern < 0) state.selectedIldaPattern = 0;
                state.ildaFrameIndex = 0;
                state.ildaLastAdvance = std::chrono::steady_clock::now();
            }
            ImGui::EndDisabled();

            // Animation controls for multi-frame patterns
            int selIdx = state.selectedIldaPattern;
            if (selIdx >= 0 && selIdx < static_cast<int>(state.ildaPatterns.size())) {
                auto& pat = state.ildaPatterns[selIdx];
                int frameCount = static_cast<int>(pat.frames.size());
                if (frameCount > 1) {
                    ImGui::Spacing();
                    ImGui::Text("Animation: %d frames", frameCount);
                    if (ImGui::Button(state.ildaPlaying ? "Pause" : "Play")) {
                        state.ildaPlaying = !state.ildaPlaying;
                        state.ildaLastAdvance = std::chrono::steady_clock::now();
                    }
                    ImGui::SameLine();
                    ImGui::Checkbox("Loop", &state.ildaLoop);
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(100.0f);
                    ImGui::DragFloat("FPS", &pat.fps, 0.5f, 1.0f, 120.0f, "%.1f");
                    // Scrubber
                    int scrubFrame = state.ildaFrameIndex;
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::SliderInt("##ildaScrub", &scrubFrame, 0, frameCount - 1,
                                         "Frame %d")) {
                        state.ildaFrameIndex = scrubFrame;
                        state.ildaLastAdvance = std::chrono::steady_clock::now();
                    }
                }
            }

            // Single colour toggle
            ImGui::Spacing();
            {
                bool wasOn = state.singleColour;
                if (wasOn) {
                    ImGui::PushStyleColor(ImGuiCol_Button,       (ImVec4)ImColor(174, 84, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(206, 115, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (ImVec4)ImColor(218, 145, 65));
                }
                if (ImGui::Button("SINGLE COLOUR MODE"))
                    state.singleColour = !state.singleColour;
                if (wasOn) ImGui::PopStyleColor(3);
            }
        }

        ImGui::Spacing();

        // Point rate
        // Compute max allowed point rate from enabled controllers
        int maxAllowedPps = 0; // 0 means no limit (no enabled controllers with known rate)
        for (auto& entry : state.controllers) {
            if (entry.enabled && entry.maxPointRate > 0) {
                if (maxAllowedPps == 0)
                    maxAllowedPps = static_cast<int>(entry.maxPointRate);
                else
                    maxAllowedPps = std::min(maxAllowedPps, static_cast<int>(entry.maxPointRate));
            }
        }

        ImGui::Separator();
        ImGui::Spacing();
        ImGui::Text("Point Rate");
        {
            float btnWidth = 52.0f, btnSpacing = 4.0f;
            static constexpr int secondRowStart = 4; // split after 30k
            for (int i = 0; i < AppState::numPresets; ++i) {
                if (i == secondRowStart) ImGui::NewLine();
                bool exceedsMax = maxAllowedPps > 0 && AppState::pointRatePresets[i].value > maxAllowedPps;
                bool selected = (state.pointRateIndex == i);
                if (exceedsMax) ImGui::BeginDisabled();
                if (selected) {
                    ImGui::PushStyleColor(ImGuiCol_Button,       (ImVec4)ImColor(174, 84, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(206, 115, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (ImVec4)ImColor(218, 145, 65));
                }
                char btnId[32];
                std::snprintf(btnId, sizeof(btnId), "%s##pps%d", AppState::pointRatePresets[i].shortLabel, i);
                if (ImGui::Button(btnId, ImVec2(btnWidth, 0))) state.pointRateIndex = i;
                if (selected) ImGui::PopStyleColor(3);
                if (exceedsMax) ImGui::EndDisabled();
                ImGui::SameLine(0, btnSpacing);
            }
            {
                bool selected = (state.pointRateIndex == AppState::customIndex);
                if (selected) {
                    ImGui::PushStyleColor(ImGuiCol_Button,       (ImVec4)ImColor(174, 84, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(206, 115, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (ImVec4)ImColor(218, 145, 65));
                }
                if (ImGui::Button("...##ppscustom", ImVec2(btnWidth, 0))) {
                    state.customPointRate = state.effectivePointRate();
                    state.pointRateIndex = AppState::customIndex;
                }
                if (selected) ImGui::PopStyleColor(3);
            }
            // Clamp selected preset down if it exceeds the controller limit
            if (maxAllowedPps > 0) {
                if (state.pointRateIndex < AppState::numPresets &&
                    AppState::pointRatePresets[state.pointRateIndex].value > maxAllowedPps) {
                    // Find the highest preset that fits
                    for (int i = AppState::numPresets - 1; i >= 0; --i) {
                        if (AppState::pointRatePresets[i].value <= maxAllowedPps) {
                            state.pointRateIndex = i;
                            break;
                        }
                    }
                }
            }
            if (state.pointRateIndex == AppState::customIndex) {
                ImGui::PushItemWidth(controlPanelWidth - 16.0f);
                ImGui::InputInt("##custompps", &state.customPointRate, 1000, 5000);
                int customMax = maxAllowedPps > 0 ? maxAllowedPps : 100000;
                state.customPointRate = std::clamp(state.customPointRate, 1000, customMax);
                ImGui::PopItemWidth();
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Output section
        {
            float degrees = state.outputSize / 100.0f * MAX_SCAN_ANGLE;
            ImGui::Text("Output  %.0f%%  (%.1f" "\xc2\xb0" ")", state.outputSize, degrees);

            // Size angle presets
            {
                struct SizePreset { const char* label; float degrees; };
                static constexpr SizePreset sizePresets[] = {
                    {"8" "\xc2\xb0", 8.0f}, {"30" "\xc2\xb0", 30.0f},
                    {"40" "\xc2\xb0", 40.0f}, {"60" "\xc2\xb0", 60.0f},
                };
                for (int i = 0; i < 4; ++i) {
                    float pct = sizePresets[i].degrees / MAX_SCAN_ANGLE * 100.0f;
                    bool selected = (std::abs(state.outputSize - pct) < 0.5f);
                    if (selected) {
                        ImGui::PushStyleColor(ImGuiCol_Button,       (ImVec4)ImColor(174, 84, 0));
                        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(206, 115, 0));
                        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (ImVec4)ImColor(218, 145, 65));
                    }
                    char btnId[32];
                    std::snprintf(btnId, sizeof(btnId), "%s##sz%d", sizePresets[i].label, i);
                    if (ImGui::Button(btnId, ImVec2(52.0f, 0))) state.outputSize = pct;
                    if (selected) ImGui::PopStyleColor(3);
                    if (i < 3) ImGui::SameLine(0, 4.0f);
                }
            }

            float totalWidth = controlPanelWidth - 16.0f;
            bool nonDefault = (std::abs(state.outputSize - 50.0f) > 0.5f ||
                               std::abs(state.outputX) > 0.5f ||
                               std::abs(state.outputY) > 0.5f);
            float dragW = (totalWidth - 80.0f) / 3.0f * 0.6f;
            ImGui::PushItemWidth(dragW);

            ImGui::AlignTextToFramePadding();
            ImGui::Text("Size"); ImGui::SameLine();
            ImGui::DragFloat("##size", &state.outputSize, 0.5f, 0.0f, 100.0f, "%.0f%%");
            state.outputSize = std::clamp(state.outputSize, 0.0f, 100.0f);

            ImGui::SameLine(0, 8); ImGui::Text("X"); ImGui::SameLine();
            ImGui::DragFloat("##xoffset", &state.outputX, 0.5f, -100.0f, 100.0f, "%.0f");
            state.outputX = std::clamp(state.outputX, -100.0f, 100.0f);

            ImGui::SameLine(0, 8); ImGui::Text("Y"); ImGui::SameLine();
            ImGui::DragFloat("##yoffset", &state.outputY, 0.5f, -100.0f, 100.0f, "%.0f");
            state.outputY = std::clamp(state.outputY, -100.0f, 100.0f);

            if (nonDefault) {
                ImGui::SameLine(0, 4);
                if (ImGui::Button(ICON_FK_UNDO "##resetSXY", ImVec2(ImGui::GetFrameHeight(), 0)))
                    state.resetOutput();
            }

            ImGui::PopItemWidth();
        }

        ImGui::Spacing();
        ImGui::Spacing();

        // Orientation
        {
            {
                float btnWidth = 60.0f;
                float btnSpacing = 4.0f;
                auto flipBtn = [&](const char* label, bool& val) {
                    bool wasOn = val;
                    if (wasOn) {
                        ImGui::PushStyleColor(ImGuiCol_Button,       (ImVec4)ImColor(174, 84, 0));
                        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(206, 115, 0));
                        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (ImVec4)ImColor(218, 145, 65));
                    }
                    if (ImGui::Button(label, ImVec2(btnWidth, 0))) val = !val;
                    if (wasOn) ImGui::PopStyleColor(3);
                };
                flipBtn("FLIP X", state.flipX);
                ImGui::SameLine(0, btnSpacing);
                flipBtn("FLIP Y", state.flipY);
            }
            ImGui::SameLine(0, 16);

            const char* orientIcons[] = {
                ICON_FK_ARROW_UP, ICON_FK_ARROW_RIGHT, ICON_FK_ARROW_DOWN, ICON_FK_ARROW_LEFT
            };
            for (int i = 0; i < 4; ++i) {
                bool selected = (state.orientation == i);
                if (selected) {
                    ImGui::PushStyleColor(ImGuiCol_Button,       (ImVec4)ImColor(174, 84, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(206, 115, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (ImVec4)ImColor(218, 145, 65));
                }
                char btnId[32];
                std::snprintf(btnId, sizeof(btnId), "%s##orient%d", orientIcons[i], i);
                if (ImGui::Button(btnId)) state.orientation = i;
                if (selected) ImGui::PopStyleColor(3);
                if (i < 3) ImGui::SameLine(0, 2);
            }
        }

        ImGui::Spacing();
        ImGui::Spacing();

        // Scanner sync
        ImGui::Text("Scanner Sync");
        ImGui::PushItemWidth(controlPanelWidth - 16.0f);
        ImGui::SliderFloat("##scansync", &state.scannerSync, 0.0f, 10.0f, "%.1f");
        ImGui::PopItemWidth();

        ImGui::Spacing();

        if (state.showAdvanced) {
            // Scanner sim PID tuning
            if (ImGui::TreeNode("Scanner Sim")) {
                ImGui::PushItemWidth(controlPanelWidth - 16.0f);
                ImGui::SliderFloat("Kp (M)##sim", &scannerSim.kpM, 0.1f, 100000.0f, "%.1f", ImGuiSliderFlags_Logarithmic);
                ImGui::SliderFloat("Kd##sim", &scannerSim.kd, 100.0f, 100000.0f, "%.0f", ImGuiSliderFlags_Logarithmic);
                ImGui::SliderFloat("Inertia##sim", &scannerSim.inertia, 0.1f, 10.0f, "%.2f");
                ImGui::SliderFloat("Friction##sim", &scannerSim.friction, 10.0f, 2000.0f, "%.0f");
                ImGui::PopItemWidth();
                ImGui::TreePop();
            }

            ImGui::Spacing();
        }

        // Scan stats (computed once per frame in updateFrameAndStats)
        {
            const ScanStats& ss = state.scanStats;
            if (state.showAdvanced && state.currentFrame.points.size() >= 3) {
                ImGui::TextDisabled("%zu pts  %.1f k" "\xc2\xb0" "/s  %.2f " "\xc2\xb0" "/ms" "\xc2\xb2",
                    ss.pointCount, ss.maxSpeed, ss.maxAccel);
            }

            // Scanner load gauge (0 – 12000 °/ms², sqrt scale)
            if (state.showAdvanced) {
                constexpr float gaugeMax = 12000.0f;
                constexpr float sqrtMax = 109.544512f; // sqrt(12000)
                float gaugeWidth = controlPanelWidth - 16.0f;
                float gaugeHeight = 14.0f;
                float tickHeight = 6.0f;

                ImVec2 p = ImGui::GetCursorScreenPos();
                ImDrawList* dl = ImGui::GetWindowDrawList();

                // Gradient bar: split into small segments for smooth green->red
                constexpr int segs = 64;
                for (int i = 0; i < segs; ++i) {
                    float t0 = (float)i / segs;
                    float t1 = (float)(i + 1) / segs;
                    auto lerpCol = [](float t) -> ImU32 {
                        float r = (t < 0.5f) ? t * 2.0f : 1.0f;
                        float g = (t < 0.5f) ? 1.0f : 1.0f - (t - 0.5f) * 2.0f;
                        return IM_COL32((int)(r * 200), (int)(g * 200), 0, 255);
                    };
                    dl->AddRectFilled(
                        ImVec2(p.x + t0 * gaugeWidth, p.y),
                        ImVec2(p.x + t1 * gaugeWidth, p.y + gaugeHeight),
                        lerpCol(t0), lerpCol(t1));
                }

                // Dark overlay beyond current value (sqrt scale)
                float val = std::min(ss.maxAccel, gaugeMax);
                float normPos = (val > 0) ? std::sqrt(val) / sqrtMax : 0.0f;
                float markerX = p.x + normPos * gaugeWidth;
                dl->AddRectFilled(
                    ImVec2(markerX, p.y),
                    ImVec2(p.x + gaugeWidth, p.y + gaugeHeight),
                    IM_COL32(0, 0, 0, 160));

                // Marker line
                dl->AddLine(
                    ImVec2(markerX, p.y),
                    ImVec2(markerX, p.y + gaugeHeight),
                    IM_COL32(255, 255, 255, 220), 2.0f);

                // Tick marks with kpps labels
                struct Tick { float value; const char* label; };
                constexpr Tick ticks[] = {
                    {410.0f, "10k"}, {1640.0f, "20k"}, {3673.0f, "30k"},
                    {6500.0f, "40k"}, {10200.0f, "50k"}
                };
                for (auto& t : ticks) {
                    float tx = p.x + (std::sqrt(t.value) / sqrtMax) * gaugeWidth;
                    dl->AddLine(
                        ImVec2(tx, p.y + gaugeHeight),
                        ImVec2(tx, p.y + gaugeHeight + tickHeight),
                        IM_COL32(180, 180, 180, 200), 1.0f);
                    ImVec2 textSize = ImGui::CalcTextSize(t.label);
                    dl->AddText(
                        ImVec2(tx - textSize.x * 0.5f, p.y + gaugeHeight + tickHeight + 1.0f),
                        IM_COL32(140, 140, 140, 200), t.label);
                }

                // Advance cursor past the gauge + ticks + labels
                ImGui::Dummy(ImVec2(gaugeWidth, gaugeHeight + tickHeight + ImGui::GetTextLineHeight() + 2.0f));
            }

            // EQ-style acceleration spectrum
            if (state.showAdvanced) {
                const SpectrumBands& spectrum = state.spectrum;

                float eqWidth = controlPanelWidth - 16.0f;
                float eqHeight = 100.0f;
                float barGap = 1.0f;
                float barWidth = (eqWidth - barGap * (EQ_NUM_BANDS - 1)) / (float)EQ_NUM_BANDS;

                ImVec2 ep = ImGui::GetCursorScreenPos();
                ImDrawList* dl = ImGui::GetWindowDrawList();

                // Dark background
                dl->AddRectFilled(ep, ImVec2(ep.x + eqWidth, ep.y + eqHeight),
                    IM_COL32(20, 20, 20, 255));

                // Fixed absolute dB scale (position spectrum in degrees)
                constexpr float dbCeiling =  30.0f;
                constexpr float dbFloor   = -30.0f;
                constexpr float dbRange   = dbCeiling - dbFloor;

                float dbValues[EQ_NUM_BANDS];
                for (int b = 0; b < EQ_NUM_BANDS; ++b) {
                    dbValues[b] = (spectrum.magnitude[b] > 1e-10f)
                        ? 20.0f * std::log10(spectrum.magnitude[b]) : -120.0f;
                }

                // Draw bars with hover tooltips
                for (int b = 0; b < EQ_NUM_BANDS; ++b) {
                    float norm = (dbValues[b] - dbFloor) / dbRange;
                    norm = std::max(0.0f, std::min(1.0f, norm));

                    float barH = norm * (eqHeight - 2.0f);
                    float bx = ep.x + (float)b * (barWidth + barGap);

                    // Color: green -> yellow -> red based on bar height
                    float r = (norm < 0.5f) ? norm * 2.0f : 1.0f;
                    float g = (norm < 0.5f) ? 1.0f : 1.0f - (norm - 0.5f) * 2.0f;
                    ImU32 col = IM_COL32((int)(r * 200), (int)(g * 200), 0, 220);

                    if (barH > 0.5f) {
                        dl->AddRectFilled(
                            ImVec2(bx, ep.y + eqHeight - barH),
                            ImVec2(bx + barWidth, ep.y + eqHeight),
                            col);
                    }

                    // Hover detection over full bar column
                    ImVec2 mousePos = ImGui::GetMousePos();
                    if (mousePos.x >= bx && mousePos.x < bx + barWidth &&
                        mousePos.y >= ep.y && mousePos.y < ep.y + eqHeight) {
                        // Highlight bar
                        dl->AddRectFilled(
                            ImVec2(bx, ep.y),
                            ImVec2(bx + barWidth, ep.y + eqHeight),
                            IM_COL32(255, 255, 255, 30));
                        // Tooltip
                        float fc = spectrum.freqCenter[b];
                        char tip[64];
                        if (fc >= 1000.0f)
                            std::snprintf(tip, sizeof(tip), "%.1f kHz  %.1f dB", fc / 1000.0f, dbValues[b]);
                        else
                            std::snprintf(tip, sizeof(tip), "%.0f Hz  %.1f dB", fc, dbValues[b]);
                        ImGui::SetTooltip("%s", tip);
                    }
                }

                // Frequency labels at fixed positions (log scale 20 Hz – 25 kHz)
                constexpr float logMin = 4.321928f;  // log2(20)
                constexpr float logMax = 14.609640f; // log2(25000)
                float tickH = 4.0f;
                struct FreqLabel { float freq; const char* label; };
                constexpr FreqLabel freqLabels[] = {
                    {100.0f, "100"}, {500.0f, "500"}, {1000.0f, "1k"},
                    {5000.0f, "5k"}, {10000.0f, "10k"}
                };
                for (auto& fl : freqLabels) {
                    float t = (std::log2(fl.freq) - logMin) / (logMax - logMin);
                    if (t < 0.0f || t > 1.0f) continue;
                    float tx = ep.x + t * eqWidth;
                    dl->AddLine(
                        ImVec2(tx, ep.y + eqHeight),
                        ImVec2(tx, ep.y + eqHeight + tickH),
                        IM_COL32(140, 140, 140, 200), 1.0f);
                    ImVec2 textSize = ImGui::CalcTextSize(fl.label);
                    dl->AddText(
                        ImVec2(tx - textSize.x * 0.5f, ep.y + eqHeight + tickH + 1.0f),
                        IM_COL32(140, 140, 140, 200), fl.label);
                }

                ImGui::Dummy(ImVec2(eqWidth, eqHeight + tickH + ImGui::GetTextLineHeight() + 2.0f));
            }

            // Scanner strain gauge (time-domain RMS acceleration)
            {
                    ImGui::Text("Scanner Load");
                    float strainVal = ss.rmsAccel;

                    constexpr float strainMax = 500.0f; // °/ms² full scale
                    constexpr float sqrtMax = 22.360680f; // sqrt(500)
                    float sgWidth = controlPanelWidth - 16.0f;
                    float sgHeight = 14.0f;
                    float sgTickH = 6.0f;

                    ImVec2 sp = ImGui::GetCursorScreenPos();
                    ImDrawList* dl = ImGui::GetWindowDrawList();

                    // Gradient bar green -> red
                    constexpr int segs = 64;
                    for (int i = 0; i < segs; ++i) {
                        float t0 = (float)i / segs;
                        float t1 = (float)(i + 1) / segs;
                        // 225 °/ms² on sqrt scale = sqrt(225)/sqrt(500) ≈ 0.67
                        auto lerpCol = [](float t) -> ImU32 {
                            // Green zone ends at 20%, fully red by 67% (≈225 °/ms²)
                            float r = (t < 0.2f) ? t * 5.0f : 1.0f;
                            float g = (t < 0.2f) ? 1.0f : std::max(0.0f, 1.0f - (t - 0.2f) / 0.47f);
                            return IM_COL32((int)(r * 200), (int)(g * 200), 0, 255);
                        };
                        dl->AddRectFilled(
                            ImVec2(sp.x + t0 * sgWidth, sp.y),
                            ImVec2(sp.x + t1 * sgWidth, sp.y + sgHeight),
                            lerpCol(t0), lerpCol(t1));
                    }

                    // Dark overlay beyond current value (sqrt scale)
                    float val = std::min(strainVal, strainMax);
                    float normPos = (val > 0) ? std::sqrt(val) / sqrtMax : 0.0f;
                    float markerX = sp.x + normPos * sgWidth;
                    dl->AddRectFilled(
                        ImVec2(markerX, sp.y),
                        ImVec2(sp.x + sgWidth, sp.y + sgHeight),
                        IM_COL32(0, 0, 0, 160));

                    // Marker line
                    dl->AddLine(
                        ImVec2(markerX, sp.y),
                        ImVec2(markerX, sp.y + sgHeight),
                        IM_COL32(255, 255, 255, 220), 2.0f);

                    // Tick marks
                    struct Tick { float value; const char* label; };
                    constexpr Tick sticks[] = {
                        {25.0f, "25"}, {75.0f, "75"}, {150.0f, "150"},
                        {275.0f, "275"}, {425.0f, "425"}
                    };
                    for (auto& t : sticks) {
                        float tx = sp.x + (std::sqrt(t.value) / sqrtMax) * sgWidth;
                        dl->AddLine(
                            ImVec2(tx, sp.y + sgHeight),
                            ImVec2(tx, sp.y + sgHeight + sgTickH),
                            IM_COL32(180, 180, 180, 200), 1.0f);
                        ImVec2 textSize = ImGui::CalcTextSize(t.label);
                        dl->AddText(
                            ImVec2(tx - textSize.x * 0.5f, sp.y + sgHeight + sgTickH + 1.0f),
                            IM_COL32(140, 140, 140, 200), t.label);
                    }

                    ImGui::Dummy(ImVec2(sgWidth, sgHeight + sgTickH + ImGui::GetTextLineHeight() + 2.0f));
                    if (strainVal > 250.0f) {
                        bool flash = strainVal > 425.0f && std::fmod(ImGui::GetTime(), 1.0) > 0.75;
                        float alpha = flash ? 0.0f : 1.0f;
                        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, alpha),
                            ICON_FK_EXCLAMATION_TRIANGLE " Warning - high scanner load");
                    } else {
                        ImGui::TextDisabled("%.1f " "\xc2\xb0" "/ms" "\xc2\xb2", strainVal);
                    }
            }
        }

        ImGui::PopItemWidth();
        ImGui::EndGroup();

        // ---- Bottom: Controller list ----
        ImGui::SetCursorScreenPos(ImVec2(previewPos.x, previewPos.y + previewSize + 12.0f));
        ImGui::BeginChild("Controllers", ImVec2(previewSize, 0), ImGuiChildFlags_None);

        if (ImGui::Button("RESCAN")) { state.discoveryRequested.store(true); state.discoveryCv.notify_one(); }
        ImGui::SameLine();
        static bool showPluginsWindow = false;
        if (ImGui::Button(ICON_FK_PLUS_CIRCLE "  Plugins")) showPluginsWindow = true;
        libera::ui::DrawPluginsWindow(&showPluginsWindow, getSettingsDir() + "/plugins");
        ImGui::SameLine();
        static bool showOscilloscope = false;
        static libera::ui::OscilloscopeState oscState;
        if (ImGui::Button(ICON_FK_SIGNAL "  Scope")) showOscilloscope = true;
        libera::ui::DrawOscilloscopeWindow(&showOscilloscope, oscState,
            state.currentFrame.points, state.effectivePointRate(),
            state.flipX, state.flipY, state.orientation);

        if (state.controllers.empty()) {
            ImGui::TextDisabled("Searching for controllers...");
        } else {
            float enableSz = ImGui::GetFrameHeight() * 0.8f;
            float frameH = ImGui::GetFrameHeight();
            float yPad = (frameH - enableSz) * 0.5f;
            ImDrawList* ctrlDrawList = ImGui::GetWindowDrawList();

            for (auto& entry : state.controllers) {
                ImGui::PushID(entry.id.c_str());

                bool busyElsewhere = !entry.enabled &&
                    (entry.usageState == core::ControllerUsageState::Active ||
                     entry.usageState == core::ControllerUsageState::BusyExclusive);
                if (busyElsewhere) ImGui::BeginDisabled();

                // Status indicator square
                {
                    ImVec2 cursor = ImGui::GetCursorScreenPos();
                    ImVec2 p(cursor.x, cursor.y + yPad);
                    ImU32 statusCol;
                    core::ControllerStatus status = core::ControllerStatus::Good;
                    if (!entry.controller || !entry.enabled) {
                        statusCol = IM_COL32(77, 77, 77, 255);
                    } else {
                        status = entry.controller->getStatus();
                        switch (status) {
                            case core::ControllerStatus::Good:   statusCol = IM_COL32(0, 255, 0, 255); break;
                            case core::ControllerStatus::Issues: statusCol = IM_COL32(255, 128, 0, 255); break;
                            case core::ControllerStatus::Error:  statusCol = IM_COL32(255, 0, 0, 255); break;
                        }
                    }
                    ctrlDrawList->AddRectFilled(p, ImVec2(p.x + enableSz, p.y + enableSz),
                                            statusCol, 2.0f);

                    ImGui::InvisibleButton("##status", ImVec2(enableSz, frameH));
                    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                        ImGui::BeginTooltip();
                        if (busyElsewhere) {
                            ImGui::Text("In use by another application");
                        } else if (!entry.controller || !entry.enabled) {
                            ImGui::Text("Not connected");
                        } else {
                            const char* statusText[] = {"Good", "Issues", "Error"};
                            ImGui::Text("Status: %s", statusText[static_cast<int>(status)]);
                            auto errors = entry.controller->getErrors();
                            if (!errors.empty()) {
                                ImGui::Separator();
                                for (auto& err : errors)
                                    ImGui::Text("%s: %llu", err.label.c_str(),
                                                static_cast<unsigned long long>(err.count));
                            }
                        }
                        ImGui::EndTooltip();
                    }
                    if (ImGui::IsItemClicked() && entry.controller)
                        entry.controller->clearErrors();

                    ImGui::SameLine();
                }

                bool wasEnabled = entry.enabled;
                {
                    char textBuf[128];
                    if (entry.controller && entry.enabled)
                        std::snprintf(textBuf, sizeof(textBuf), "%s  %u pps", entry.label.c_str(), entry.controller->getPointRate());
                    else
                        std::snprintf(textBuf, sizeof(textBuf), "%s", entry.label.c_str());
                    toggleButton("##enable", &entry.enabled, entry.connecting, enableSz, textBuf);
                }

                if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                    ImGui::BeginTooltip();
                    ImGui::Text("Type: %s", entry.type.c_str());
                    if (entry.maxPointRate > 0) ImGui::Text("Max: %u pps", entry.maxPointRate);
                    ImGui::Text("ID: %s", entry.id.c_str());
                    if (busyElsewhere) ImGui::Text("In use by another application");
                    ImGui::EndTooltip();
                }

                if (busyElsewhere) ImGui::EndDisabled();

                if (entry.enabled && !wasEnabled && !entry.connecting)
                    startAsyncConnect(state, entry);
                else if (!entry.enabled && wasEnabled)
                    disconnectController(entry);

                ImGui::PopID();
            }
        }

        ImGui::EndChild();

        // Logo: top-left of preview, on foreground so it's always visible
        drawLogo(io, ImVec2(previewPos.x + 16.0f, previewPos.y + 10.0f),
                 previewSize - 22.0f, false, ImGui::GetForegroundDrawList());

        ImGui::End();

        app.endFrame();
    }

    // Capture window geometry before saving
    app.getWindowGeometry(state.windowX, state.windowY, state.windowWidth, state.windowHeight);
    saveSettings(state);

    state.discoveryRunning.store(false);
    state.discoveryCv.notify_one();
    libera::core::timedJoin(state.discoveryThread, state.discoveryFinished,
                            std::chrono::milliseconds(3000), "discoveryThread");
    for (auto& entry : state.controllers) disconnectController(entry);
    state.liberaSystem.shutdown();

    app.shutdown();
    return 0;
}
