// Libera Lab — laser controller service & test utility
//
// Cross-platform (macOS / Windows / Linux) using:
//   GLFW + OpenGL3 for windowing
//   Dear ImGui for the UI
//   libera-core for controller discovery and output

#include "libera.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "fonts/IconsForkAwesome.h"
#include "ILDAParser.h"

// Compressed font data (linked from fonts/*.cpp)
extern const unsigned int RobotoMedium_compressed_size;
extern const unsigned int RobotoMedium_compressed_data[];
extern const unsigned int RobotoBold_compressed_size;
extern const unsigned int RobotoBold_compressed_data[];
extern const unsigned int ForkAwesome_compressed_size;
extern const unsigned int ForkAwesome_compressed_data[];

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <GLFW/glfw3.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
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

static constexpr int NUM_PATTERNS = 4;
static const char* patternNames[NUM_PATTERNS] = {
    "White Square", "White Circle", "RGBW Square", "Hot Corners"
};

enum class OutputMode { Shape, Point, Pattern };

static constexpr int NUM_POINT_PATTERNS = 5;
static const char* pointPatternNames[NUM_POINT_PATTERNS] = {
    "Single", "2 Vert", "2 Horiz", "4 Grid", "8 Row"
};

struct ILDAPattern {
    std::string name;             // filename without extension
    std::string path;             // full path
    std::vector<ILDAPoint> points; // first frame only
};

struct ControllerEntry {
    std::string id;
    std::string label;
    std::string type;
    std::uint32_t maxPointRate = 0;
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
    int pointRateIndex = 2;
    int customPointRate = 30000;
    float outputSize = 50.0f;
    float outputX = 0.0f;
    float outputY = 0.0f;

    // ILDA patterns
    std::vector<ILDAPattern> ildaPatterns;
    int selectedIldaPattern = 0;

    // Window geometry (saved/restored)
    int windowWidth = 1100;
    int windowHeight = 700;
    int windowX = -1; // -1 = let OS decide
    int windowY = -1;

    bool flipX = false;
    bool flipY = false;
    int orientation = 0;
    float scannerSync = 2.0f; // in 1/10,000s units (0.1ms). Default 2.0 = 0.2ms

    bool draggingOutput = false;
    bool resizingOutput = false;
    int resizingCorner = -1;

    std::set<std::string> savedEnabledControllers;

    System liberaSystem;
    std::vector<ControllerEntry> controllers;
    std::thread discoveryThread;
    std::atomic<bool> discoveryRunning{false};
    std::mutex discoveredMutex;
    std::vector<DiscoveredInfo> latestDiscovered;
    std::atomic<bool> discoveryResultReady{false};
    std::atomic<bool> discoveryRequested{false};

    struct PointRatePreset {
        const char* label;
        const char* shortLabel;
        int value;
    };
    static constexpr PointRatePreset pointRatePresets[] = {
        {"10,000 pps", "10k",  10000},
        {"20,000 pps", "20k",  20000},
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

static core::Frame makeSquareFrame(float r, float g, float b, float size, float cx, float cy) {
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
            frame.points.push_back({x, y, r, g, b});
        }
    }
    // Close the loop
    frame.points.push_back(frame.points.front());
    return frame;
}

static core::Frame makeCircleFrame(float r, float g, float b, float size, float cx, float cy) {
    core::Frame frame;
    const float radius = size * 0.8f;
    constexpr int pointCount = 400;
    frame.points.reserve(pointCount);
    for (int i = 0; i < pointCount; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(pointCount);
        float angle = t * TAU;
        frame.points.push_back({
            cx + radius * std::cos(angle),
            cy + radius * std::sin(angle),
            r, g, b
        });
    }
    return frame;
}

// RGBW Square: white top, red left, green bottom, blue right
static core::Frame makeRGBWSquareFrame(float r, float g, float b, float size, float cx, float cy) {
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
        {0, g, 0},   // side 0 (bottom): green
        {r, 0, 0},   // side 1 (right): red
        {r, r, r},   // side 2 (top): white
        {0, 0, b},   // side 3 (left): blue
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
static core::Frame makeHotCornersFrame(float r, float g, float b, float size, float cx, float cy) {
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
            frame.points.push_back({corners[side][0], corners[side][1], r, g, b});
        }
        // Move quickly along edge
        for (int i = 0; i < edgePoints; ++i) {
            float t = static_cast<float>(i + 1) / static_cast<float>(edgePoints);
            float x = corners[side][0] + t * (corners[next][0] - corners[side][0]);
            float y = corners[side][1] + t * (corners[next][1] - corners[side][1]);
            frame.points.push_back({x, y, r, g, b});
        }
    }
    // Already closes: last edge ends at corner 0 = first dwell point
    return frame;
}

// Generate point positions for a given point pattern
static std::vector<std::pair<float,float>> getPointPositions(int patternIndex, float cx, float cy, float spacing) {
    std::vector<std::pair<float,float>> pts;
    float s = spacing * 0.3f; // spacing between dots
    switch (patternIndex) {
        case 0: // Single
            pts.push_back({cx, cy});
            break;
        case 1: // 2 Vertical
            pts.push_back({cx, cy - s});
            pts.push_back({cx, cy + s});
            break;
        case 2: // 2 Horizontal
            pts.push_back({cx - s, cy});
            pts.push_back({cx + s, cy});
            break;
        case 3: // 4 Grid
            pts.push_back({cx - s, cy - s});
            pts.push_back({cx + s, cy - s});
            pts.push_back({cx + s, cy + s});
            pts.push_back({cx - s, cy + s});
            break;
        case 4: // 8 Row — left-to-right then right-to-left
        {
            float halfW = spacing * 0.8f;
            // Left to right
            for (int i = 0; i < 8; ++i) {
                float t = (static_cast<float>(i) / 7.0f - 0.5f) * 2.0f * halfW;
                pts.push_back({cx + t, cy});
            }
            // Right to left (skip endpoints to avoid double-dwell)
            for (int i = 6; i >= 1; --i) {
                float t = (static_cast<float>(i) / 7.0f - 0.5f) * 2.0f * halfW;
                pts.push_back({cx + t, cy});
            }
            break;
        }
        default:
            pts.push_back({cx, cy});
            break;
    }
    return pts;
}

static core::Frame makePointFrame(int patternIndex, float r, float g, float b,
                                   float cx, float cy, float size, float dutyCycle) {
    core::Frame frame;
    auto positions = getPointPositions(patternIndex, cx, cy, size);
    int numDots = static_cast<int>(positions.size());
    int dwellPerDot = std::max(4, 100 / numDots);
    int onPoints = std::max(1, static_cast<int>(std::round(dutyCycle / 100.0f * dwellPerDot)));
    int transitPoints = (numDots > 1) ? 20 : 0; // smooth blank path between dots

    frame.points.reserve((dwellPerDot + transitPoints) * numDots);
    for (int d = 0; d < numDots; ++d) {
        auto [px, py] = positions[d];

        // Dwell on this dot (with duty cycle)
        for (int i = 0; i < dwellPerDot; ++i) {
            if (i < onPoints) {
                frame.points.push_back({px, py, r, g, b});
            } else {
                frame.points.push_back({px, py, 0, 0, 0});
            }
        }

        // Smooth blank transit to next dot
        if (numDots > 1) {
            auto [nx, ny] = positions[(d + 1) % numDots];
            for (int i = 0; i < transitPoints; ++i) {
                float t = static_cast<float>(i + 1) / static_cast<float>(transitPoints + 1);
                float tx = px + t * (nx - px);
                float ty = py + t * (ny - py);
                frame.points.push_back({tx, ty, 0, 0, 0});
            }
        }
    }
    return frame;
}

static core::Frame makeShapeFrame(int patternIndex, float r, float g, float b,
                                   float size, float cx, float cy) {
    switch (patternIndex) {
        case 0: return makeSquareFrame(r, g, b, size, cx, cy);
        case 1: return makeCircleFrame(r, g, b, size, cx, cy);
        case 2: return makeRGBWSquareFrame(r, g, b, size, cx, cy);
        case 3: return makeHotCornersFrame(r, g, b, size, cx, cy);
    }
    return makeSquareFrame(r, g, b, size, cx, cy);
}

// ---------------------------------------------------------------------------
// ILDA pattern loading and frame conversion
// ---------------------------------------------------------------------------

static std::string getPatternsDir(const char* argv0) {
    // Look for patterns/ next to the executable
    std::string exePath(argv0);
    auto lastSlash = exePath.find_last_of("/\\");
    std::string dir = (lastSlash != std::string::npos) ? exePath.substr(0, lastSlash) : ".";
    return dir + "/patterns";
}

static void loadILDAPatterns(AppState& state, const std::string& dir) {
    state.ildaPatterns.clear();

    // Scan directory for .ild files
#ifdef _WIN32
    WIN32_FIND_DATAA fd;
    HANDLE hFind = FindFirstFileA((dir + "\\*.ild").c_str(), &fd);
    if (hFind == INVALID_HANDLE_VALUE) return;
    do {
        std::string filename = fd.cFileName;
#else
    // POSIX: use opendir
    auto* d = opendir(dir.c_str());
    if (!d) return;
    struct dirent* entry;
    while ((entry = readdir(d)) != nullptr) {
        std::string filename = entry->d_name;
        if (filename.size() < 4) continue;
        std::string ext = filename.substr(filename.size() - 4);
        // Case-insensitive .ild check
        for (auto& c : ext) c = static_cast<char>(::tolower(c));
        if (ext != ".ild") continue;
#endif
        std::string fullPath = dir + "/" + filename;
        auto frames = ILDAParser::load(fullPath);
        if (!frames.empty() && !frames[0].empty()) {
            // Strip extension for display name
            std::string name = filename;
            auto dot = name.find_last_of('.');
            if (dot != std::string::npos) name = name.substr(0, dot);

            state.ildaPatterns.push_back({name, fullPath, frames[0]});
        }
#ifdef _WIN32
    } while (FindNextFileA(hFind, &fd));
    FindClose(hFind);
#else
    }
    closedir(d);
#endif

    // Sort by name
    std::sort(state.ildaPatterns.begin(), state.ildaPatterns.end(),
              [](const ILDAPattern& a, const ILDAPattern& b) { return a.name < b.name; });
}

// Convert ILDA points to a libera Frame, applying brightness/RGB and size/offset
static core::Frame makeILDAFrame(const std::vector<ILDAPoint>& ildaPoints,
                                  float r, float g, float b,
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
            // Extract ILDA colour and multiply by user RGB/brightness
            float ir = static_cast<float>((ip.color >> 16) & 0xFF) / 255.0f;
            float ig = static_cast<float>((ip.color >> 8) & 0xFF) / 255.0f;
            float ib = static_cast<float>(ip.color & 0xFF) / 255.0f;
            frame.points.push_back({nx, ny, ir * r, ig * g, ib * b});
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
                               bool flipYForPreview = false) {
    core::Frame frame = makeShapeFrame(patternIndex, 1.0f, 1.0f, 1.0f, size, cx, cy);

    auto mapX = [&](float x) { return rectPos.x + (x + 1.0f) * 0.5f * rectSize.x; };
    auto mapY = [&](float y) {
        if (flipYForPreview) y = -y;
        return rectPos.y + (y + 1.0f) * 0.5f * rectSize.y;
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
                            float scale = 1.0f, float cx = 0.0f, float cy = 0.0f) {
    if (points.empty()) return;

    auto mapX = [&](float x) { return rectPos.x + (x + 1.0f) * 0.5f * rectSize.x; };
    auto mapY = [&](float y) {
        if (flipY) y = -y;
        return rectPos.y + (y + 1.0f) * 0.5f * rectSize.y;
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
// Jerk rate calculation
// ---------------------------------------------------------------------------

struct ScanStats {
    float maxSpeed = 0.0f;  // k°/s
    float maxAccel = 0.0f;  // k°/s²
    std::size_t pointCount = 0;
};

static ScanStats calculateScanStats(const core::Frame& frame, float pointRate) {
    ScanStats stats;
    stats.pointCount = frame.points.size();
    if (frame.points.size() < 3 || pointRate <= 0) return stats;

    constexpr float halfAngle = MAX_SCAN_ANGLE * 0.5f; // ±30°
    float dt = 1.0f / pointRate;

    float prevVx = 0, prevVy = 0;
    bool havePrevVel = false;

    for (std::size_t i = 1; i < frame.points.size(); ++i) {
        // Position delta in degrees
        float dx = (frame.points[i].x - frame.points[i - 1].x) * halfAngle;
        float dy = (frame.points[i].y - frame.points[i - 1].y) * halfAngle;

        // Velocity in °/s
        float vx = dx / dt, vy = dy / dt;
        float speed = std::sqrt(vx * vx + vy * vy);
        if (speed > stats.maxSpeed) stats.maxSpeed = speed;

        if (havePrevVel) {
            // Acceleration in °/s²
            float ax = (vx - prevVx) / dt;
            float ay = (vy - prevVy) / dt;
            float accel = std::sqrt(ax * ax + ay * ay);
            if (accel > stats.maxAccel) stats.maxAccel = accel;
        }

        prevVx = vx;
        prevVy = vy;
        havePrevVel = true;
    }

    // Convert: speed to k°/s, accel to °/ms² (= °/s² ÷ 1,000,000)
    stats.maxSpeed /= 1000.0f;
    stats.maxAccel /= 1000000.0f;

    return stats;
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

    float previewBrightness = state.armed ? 1.0f : 0.47f;

    if (state.outputMode == 0) {
        // Shape mode
        drawPatternInRect(drawList, state.patternIndex, previewSize, previewCx, previewCy,
                          pos, size, previewBrightness, true);
    } else if (state.outputMode == 1) {
        // Point mode — draw dots
        auto bMapX = [&](float x) { return pos.x + (x + 1.0f) * 0.5f * size.x; };
        auto bMapY = [&](float y) { return pos.y + ((-y) + 1.0f) * 0.5f * size.y; };
        uint8_t bv = static_cast<uint8_t>(previewBrightness * 255);
        auto positions = getPointPositions(state.pointPatternIndex, previewCx, previewCy, previewSize);
        for (auto& [px, py] : positions) {
            float dotX = bMapX(px);
            float dotY = bMapY(py);
            drawList->AddCircleFilled(ImVec2(dotX, dotY), 4.0f, IM_COL32(bv, bv, bv, 255));
            drawList->AddLine(ImVec2(dotX - 8, dotY), ImVec2(dotX + 8, dotY), IM_COL32(80, 80, 80, 140));
            drawList->AddLine(ImVec2(dotX, dotY - 8), ImVec2(dotX, dotY + 8), IM_COL32(80, 80, 80, 140));
        }
    } else if (state.outputMode == 2) {
        // Pattern mode — draw selected ILDA pattern with size/offset
        int idx = state.selectedIldaPattern;
        if (idx >= 0 && idx < static_cast<int>(state.ildaPatterns.size())) {
            float brightness = state.armed ? 1.0f : 0.47f;
            drawILDAInRect(drawList, state.ildaPatterns[idx].points, pos, size, true,
                           brightness, previewSize * 0.8f, previewCx, previewCy);
        }
    }

    if (!state.armed) {
        const char* text = "DISARMED";
        ImVec2 textSize = ImGui::CalcTextSize(text);
        drawList->AddText(
            ImVec2(pos.x + (size.x - textSize.x) * 0.5f, pos.y + size.y - textSize.y - 8.0f),
            IM_COL32(100, 100, 100, 255), text);
    }

    // Output bounding box
    float s = previewSize * 0.8f;
    auto mapX = [&](float x) { return pos.x + (x + 1.0f) * 0.5f * size.x; };
    auto mapY = [&](float y) { return pos.y + ((-y) + 1.0f) * 0.5f * size.y; };

    float rectLeft   = mapX(previewCx - s);
    float rectTop    = mapY(previewCy + s);
    float rectRight  = mapX(previewCx + s);
    float rectBottom = mapY(previewCy - s);
    if (rectTop > rectBottom) std::swap(rectTop, rectBottom);
    if (rectLeft > rectRight) std::swap(rectLeft, rectRight);

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
                state.latestDiscovered.push_back({d->idValue(), d->labelValue(), d->type(), d->maxPointRate()});
            state.discoveryResultReady.store(true);
        }
        for (int i = 0; i < 20 && state.discoveryRunning.load(); ++i) {
            if (state.discoveryRequested.load()) { state.discoveryRequested.store(false); break; }
            std::this_thread::sleep_for(100ms);
        }
    }
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
        for (auto& d : discovered) { if (d.id == entry.id) { stillPresent = true; break; } }
        if (!stillPresent && entry.controller) { entry.controller.reset(); entry.enabled = false; }
    }
    for (auto& d : discovered) {
        bool exists = false;
        for (auto& entry : state.controllers) { if (entry.id == d.id) { exists = true; break; } }
        if (!exists) {
            bool shouldAutoConnect = state.savedEnabledControllers.count(d.id) > 0;
            state.controllers.push_back({d.id, d.label, d.type, d.maxPointRate, shouldAutoConnect, false, nullptr, {}});
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

static core::Frame buildCurrentFrame(const AppState& state) {
    float r, g, b;
    state.effectiveRGB(r, g, b);
    float sz = state.outputSize / 100.0f;
    float cx = state.outputX / 100.0f;
    float cy = state.outputY / 100.0f;

    if (state.outputMode == 1) {
        return makePointFrame(state.pointPatternIndex, r, g, b, cx, cy, sz, state.dutyCycle);
    } else if (state.outputMode == 2) {
        int idx = state.selectedIldaPattern;
        if (idx >= 0 && idx < static_cast<int>(state.ildaPatterns.size()))
            return makeILDAFrame(state.ildaPatterns[idx].points, r, g, b, sz * 0.8f, cx, cy);
        return makeShapeFrame(0, r, g, b, sz, cx, cy); // fallback
    }
    return makeShapeFrame(state.patternIndex, r, g, b, sz, cx, cy);
}

// ---------------------------------------------------------------------------
// Frame sending
// ---------------------------------------------------------------------------

static void sendFramesToControllers(AppState& state) {
    core::Frame frame = buildCurrentFrame(state);


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

static bool toggleButton(const char* label, bool* v, bool showConnecting = false) {
    ImGui::PushID(label);
    bool clicked = false;
    bool on = *v;

    ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    float sz = ImGui::GetFrameHeight();
    if (ImGui::InvisibleButton("##toggle", ImVec2(sz, sz))) {
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

    ImGui::PopID();
    return clicked;
}

// ---------------------------------------------------------------------------
// Draw the LIBERA LAB logo
// ---------------------------------------------------------------------------

static void drawLogo(ImGuiIO& io, ImVec2 pos, float panelWidth) {
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    ImFont* font = io.Fonts->Fonts[2];
    float fontSize = font->FontSize * io.FontGlobalScale;

    const char* t1 = "LIBERA";
    const char* t2 = "LAB";
    ImVec2 s1 = font->CalcTextSizeA(fontSize, FLT_MAX, 0, t1);
    ImVec2 s2 = font->CalcTextSizeA(fontSize, FLT_MAX, 0, t2);
    float totalW = s1.x + 4.0f + s2.x;
    float x = pos.x + panelWidth - totalW - 16.0f;
    float y = pos.y;

    drawList->AddText(font, fontSize, ImVec2(x, y),
                      IM_COL32(66, 150, 250, 255), t1);
    drawList->AddText(font, fontSize, ImVec2(x + s1.x + 4.0f, y),
                      IM_COL32(153, 153, 153, 255), t2);
}


// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int /*argc*/, char* argv[]) {
    if (!glfwInit()) {
        std::fprintf(stderr, "Failed to initialise GLFW\n");
        return 1;
    }

#ifdef __APPLE__
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    const char* glslVersion = "#version 150";
#else
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    const char* glslVersion = "#version 130";
#endif

    // Load settings early so we can restore window geometry
    AppState state;
    loadSettings(state);

    GLFWwindow* window = glfwCreateWindow(state.windowWidth, state.windowHeight, "Libera Lab", nullptr, nullptr);
    if (!window) { std::fprintf(stderr, "Failed to create window\n"); glfwTerminate(); return 1; }
    if (state.windowX >= 0 && state.windowY >= 0)
        glfwSetWindowPos(window, state.windowX, state.windowY);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;

    float xscale = 1.0f, yscale = 1.0f;
    glfwGetWindowContentScale(window, &xscale, &yscale);
    float dpiScale = xscale;

    float baseFontSize = 16.0f * dpiScale, mediumFontSize = 20.0f * dpiScale, largeFontSize = 28.0f * dpiScale;
    ImFontConfig fontConfig; fontConfig.OversampleH = 2; fontConfig.OversampleV = 2;
    ImFontConfig mergeConfig; mergeConfig.MergeMode = true; mergeConfig.OversampleH = 2; mergeConfig.OversampleV = 2;
    static const ImWchar iconRanges[] = { ICON_MIN_FK, ICON_MAX_FK, 0 };

    io.Fonts->AddFontFromMemoryCompressedTTF(RobotoMedium_compressed_data, RobotoMedium_compressed_size, baseFontSize, &fontConfig);
    mergeConfig.GlyphMinAdvanceX = baseFontSize;
    io.Fonts->AddFontFromMemoryCompressedTTF(ForkAwesome_compressed_data, ForkAwesome_compressed_size, baseFontSize, &mergeConfig, iconRanges);
    io.Fonts->AddFontFromMemoryCompressedTTF(RobotoBold_compressed_data, RobotoBold_compressed_size, mediumFontSize, &fontConfig);
    mergeConfig.GlyphMinAdvanceX = mediumFontSize;
    io.Fonts->AddFontFromMemoryCompressedTTF(ForkAwesome_compressed_data, ForkAwesome_compressed_size, mediumFontSize, &mergeConfig, iconRanges);
    io.Fonts->AddFontFromMemoryCompressedTTF(RobotoBold_compressed_data, RobotoBold_compressed_size, largeFontSize, &fontConfig);
    mergeConfig.GlyphMinAdvanceX = largeFontSize;
    io.Fonts->AddFontFromMemoryCompressedTTF(ForkAwesome_compressed_data, ForkAwesome_compressed_size, largeFontSize, &mergeConfig, iconRanges);
    io.Fonts->Build();
    io.FontGlobalScale = 1.0f / dpiScale;

    // ---- Style ----
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f; style.WindowBorderSize = 0.0f; style.IndentSpacing = 0.0f;
    style.ItemSpacing = ImVec2(6.0f, 6.0f); style.ItemInnerSpacing = ImVec2(6.0f, 6.0f);
    style.WindowMinSize = ImVec2(10.0f, 10.0f); style.FramePadding = ImVec2(8.0f, 6.0f);
    style.FrameRounding = 2.0f; style.GrabRounding = 1.0f; style.GrabMinSize = 16.0f; style.PopupRounding = 8.0f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text]                   = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
    colors[ImGuiCol_TextDisabled]           = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
    colors[ImGuiCol_WindowBg]               = ImVec4(0.0f, 0.0f, 0.0f, 0.94f);
    colors[ImGuiCol_ChildBg]                = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_PopupBg]                = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
    colors[ImGuiCol_Border]                 = ImVec4(0.43f, 0.43f, 0.50f, 0.50f);
    colors[ImGuiCol_BorderShadow]           = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_FrameBg]                = ImVec4(0.16f, 0.29f, 0.48f, 0.4f);
    colors[ImGuiCol_FrameBgHovered]         = ImVec4(0.26f, 0.59f, 0.98f, 0.40f);
    colors[ImGuiCol_FrameBgActive]          = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
    colors[ImGuiCol_TitleBg]                = ImVec4(24.0f/255.0f, 43.0f/255.0f, 72.0f/255.0f, 230.0f/255.0f);
    colors[ImGuiCol_TitleBgActive]          = ImVec4(0.16f, 0.29f, 0.48f, 0.95f);
    colors[ImGuiCol_TitleBgCollapsed]       = ImVec4(0.00f, 0.00f, 0.00f, 0.51f);
    colors[ImGuiCol_MenuBarBg]              = ImVec4(0.14f, 0.14f, 0.14f, 1.00f);
    colors[ImGuiCol_ScrollbarBg]            = ImVec4(0.02f, 0.02f, 0.02f, 0.53f);
    colors[ImGuiCol_ScrollbarGrab]          = ImVec4(0.31f, 0.31f, 0.31f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabHovered]   = ImVec4(0.41f, 0.41f, 0.41f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabActive]    = ImVec4(0.51f, 0.51f, 0.51f, 1.00f);
    colors[ImGuiCol_CheckMark]              = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    colors[ImGuiCol_SliderGrab]             = ImVec4(0.24f, 0.52f, 0.88f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]       = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    colors[ImGuiCol_Button]                 = ImVec4(0.26f, 0.59f, 0.98f, 0.40f);
    colors[ImGuiCol_ButtonHovered]          = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    colors[ImGuiCol_ButtonActive]           = ImVec4(0.06f, 0.53f, 0.98f, 1.00f);
    colors[ImGuiCol_Header]                 = ImVec4(0.26f, 0.59f, 0.98f, 0.31f);
    colors[ImGuiCol_HeaderHovered]          = ImVec4(0.26f, 0.59f, 0.98f, 0.50f);
    colors[ImGuiCol_HeaderActive]           = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    colors[ImGuiCol_Separator]              = ImVec4(0.43f, 0.43f, 0.50f, 0.10f);
    colors[ImGuiCol_SeparatorHovered]       = ImVec4(0.10f, 0.40f, 0.75f, 0.78f);
    colors[ImGuiCol_SeparatorActive]        = ImVec4(0.10f, 0.40f, 0.75f, 1.00f);
    colors[ImGuiCol_ResizeGrip]             = ImVec4(0.26f, 0.59f, 0.98f, 0.20f);
    colors[ImGuiCol_ResizeGripHovered]      = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
    colors[ImGuiCol_ResizeGripActive]       = ImVec4(0.26f, 0.59f, 0.98f, 0.95f);
    colors[ImGuiCol_PlotLines]              = ImVec4(0.61f, 0.61f, 0.61f, 1.00f);
    colors[ImGuiCol_PlotLinesHovered]       = ImVec4(1.00f, 0.43f, 0.35f, 1.00f);
    colors[ImGuiCol_PlotHistogram]          = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
    colors[ImGuiCol_PlotHistogramHovered]   = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
    colors[ImGuiCol_TableHeaderBg]          = ImVec4(0.19f, 0.19f, 0.20f, 1.00f);
    colors[ImGuiCol_TableBorderStrong]      = ImVec4(0.31f, 0.31f, 0.35f, 1.00f);
    colors[ImGuiCol_TableBorderLight]       = ImVec4(0.23f, 0.23f, 0.25f, 1.00f);
    colors[ImGuiCol_TableRowBg]             = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_TableRowBgAlt]          = ImVec4(1.00f, 1.00f, 1.00f, 0.06f);
    colors[ImGuiCol_TextSelectedBg]         = ImVec4(0.26f, 0.59f, 0.98f, 0.35f);
    colors[ImGuiCol_DragDropTarget]         = ImVec4(1.00f, 1.00f, 0.00f, 0.90f);
    colors[ImGuiCol_NavHighlight]           = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    colors[ImGuiCol_NavWindowingHighlight]  = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
    colors[ImGuiCol_NavWindowingDimBg]      = ImVec4(0.80f, 0.80f, 0.80f, 0.20f);
    colors[ImGuiCol_ModalWindowDimBg]       = ImVec4(0.0f, 0.0f, 0.0f, 0.4f);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glslVersion);

    // Load ILDA patterns from patterns/ folder next to executable
    loadILDAPatterns(state, getPatternsDir(argv[0]));

    state.discoveryRunning.store(true);
    state.discoveryThread = std::thread(discoveryThreadFunc, std::ref(state));

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        applyDiscoveryResults(state);
        pollAsyncConnections(state);
        sendFramesToControllers(state);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

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

        // Logo at top-left of window (drawn on draw list, doesn't affect layout)
        ImVec2 logoPos = ImGui::GetCursorScreenPos();
        drawLogo(io, logoPos, windowWidth);

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
                    *btns[i].enabled = !*btns[i].enabled;
                ImGui::PopStyleColor(3);
                if (i < 2) ImGui::SameLine(0, spacing);
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Brightness slider (no label)
        ImGui::PushItemWidth(controlPanelWidth - 16.0f);
        ImGui::SliderFloat("##brightness", &state.brightness, 0.0f, 100.0f, "%.0f%%");
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
            ImGui::Text("Test Pattern");
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
                    if (ImGui::InvisibleButton(btnId, ImVec2(thumbSize, thumbSize)))
                        state.patternIndex = i;

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
                    uint8_t tv = selected ? 255 : 150;
                    auto positions = getPointPositions(i, 0.0f, 0.0f, 1.0f);
                    for (auto& [px, py] : positions) {
                        float dx = thumbPos.x + (px + 1.0f) * 0.5f * thumbSize;
                        float dy = thumbPos.y + ((-py) + 1.0f) * 0.5f * thumbSize;
                        ptDrawList->AddCircleFilled(ImVec2(dx, dy), 3.0f, IM_COL32(tv, tv, tv, 255));
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
                    if (ImGui::InvisibleButton(btnId, ImVec2(thumbSize, thumbSize)))
                        state.pointPatternIndex = i;

                    bool endOfRow = ((i + 1) % 4 == 0);
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

                    drawILDAInRect(ildaDrawList, state.ildaPatterns[i].points,
                                   thumbPos, ImVec2(thumbSize, thumbSize), false,
                                   selected ? 1.0f : 0.6f);

                    // Label
                    const char* name = state.ildaPatterns[i].name.c_str();
                    ImVec2 labelSize = ImGui::CalcTextSize(name);
                    ildaDrawList->AddText(
                        ImVec2(thumbPos.x + std::max(0.0f, (thumbSize - labelSize.x) * 0.5f),
                               thumbPos.y + thumbSize + 2.0f),
                        selected ? IM_COL32(255, 255, 255, 255) : IM_COL32(150, 150, 150, 255),
                        name);

                    ImGui::SetCursorScreenPos(thumbPos);
                    char btnId[32];
                    std::snprintf(btnId, sizeof(btnId), "##ilda%d", i);
                    if (ImGui::InvisibleButton(btnId, ImVec2(thumbSize, thumbSize)))
                        state.selectedIldaPattern = i;

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
        }

        ImGui::Spacing();

        // Point rate
        ImGui::Text("Point Rate");
        {
            float btnWidth = 52.0f, btnSpacing = 4.0f;
            for (int i = 0; i < AppState::numPresets; ++i) {
                bool selected = (state.pointRateIndex == i);
                if (selected) {
                    ImGui::PushStyleColor(ImGuiCol_Button,       (ImVec4)ImColor(174, 84, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(206, 115, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (ImVec4)ImColor(218, 145, 65));
                }
                char btnId[32];
                std::snprintf(btnId, sizeof(btnId), "%s##pps%d", AppState::pointRatePresets[i].shortLabel, i);
                if (ImGui::Button(btnId, ImVec2(btnWidth, 0))) state.pointRateIndex = i;
                if (selected) ImGui::PopStyleColor(3);
                ImGui::SameLine(0, btnSpacing);
            }
            {
                bool selected = (state.pointRateIndex == AppState::customIndex);
                if (selected) {
                    ImGui::PushStyleColor(ImGuiCol_Button,       (ImVec4)ImColor(174, 84, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(206, 115, 0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  (ImVec4)ImColor(218, 145, 65));
                }
                if (ImGui::Button("...##ppscustom", ImVec2(btnWidth, 0))) state.pointRateIndex = AppState::customIndex;
                if (selected) ImGui::PopStyleColor(3);
            }
            if (state.pointRateIndex == AppState::customIndex) {
                ImGui::PushItemWidth(controlPanelWidth - 16.0f);
                ImGui::InputInt("##custompps", &state.customPointRate, 1000, 5000);
                state.customPointRate = std::clamp(state.customPointRate, 1000, 100000);
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
            ImGui::SameLine(controlPanelWidth - 70.0f);
            if (ImGui::SmallButton("RESET")) state.resetOutput();

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
            float dragW = (totalWidth - 60.0f) / 3.0f;
            ImGui::PushItemWidth(dragW);

            ImGui::Text("Size"); ImGui::SameLine();
            ImGui::DragFloat("##size", &state.outputSize, 0.5f, 0.0f, 100.0f, "%.0f%%");
            state.outputSize = std::clamp(state.outputSize, 0.0f, 100.0f);

            ImGui::SameLine(0, 8); ImGui::Text("X"); ImGui::SameLine();
            ImGui::DragFloat("##xoffset", &state.outputX, 0.5f, -100.0f, 100.0f, "%.0f%%");
            state.outputX = std::clamp(state.outputX, -100.0f, 100.0f);

            ImGui::SameLine(0, 8); ImGui::Text("Y"); ImGui::SameLine();
            ImGui::DragFloat("##yoffset", &state.outputY, 0.5f, -100.0f, 100.0f, "%.0f%%");
            state.outputY = std::clamp(state.outputY, -100.0f, 100.0f);

            ImGui::PopItemWidth();
        }

        ImGui::Spacing();

        // Orientation
        {
            ImGui::Checkbox("Flip X", &state.flipX);
            ImGui::SameLine();
            ImGui::Checkbox("Flip Y", &state.flipY);
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

        // Scanner sync
        ImGui::Text("Scanner Sync");
        ImGui::PushItemWidth(controlPanelWidth - 16.0f);
        ImGui::SliderFloat("##scansync", &state.scannerSync, 0.0f, 10.0f, "%.1f");
        ImGui::PopItemWidth();

        ImGui::Spacing();

        // Scan stats for current pattern
        {
            core::Frame statsFrame = buildCurrentFrame(state);
            ScanStats ss;
            if (statsFrame.points.size() >= 3) {
                float pps = static_cast<float>(state.effectivePointRate());
                ss = calculateScanStats(statsFrame, pps);
                ImGui::TextDisabled("%zu pts  %.1f k" "\xc2\xb0" "/s  %.2f " "\xc2\xb0" "/ms" "\xc2\xb2",
                    ss.pointCount, ss.maxSpeed, ss.maxAccel);
            }

            // Scanner load gauge (0 – 12000 °/ms², sqrt scale)
            {
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
        }

        ImGui::PopItemWidth();
        ImGui::EndGroup();

        // ---- Bottom: Controller list ----
        ImGui::SetCursorScreenPos(ImVec2(previewPos.x, previewPos.y + previewSize + 12.0f));
        ImGui::BeginChild("Controllers", ImVec2(previewSize, 0), ImGuiChildFlags_Border);

        ImGui::Text("Discovered Controllers");
        ImGui::SameLine();
        if (ImGui::SmallButton("RESCAN")) state.discoveryRequested.store(true);
        ImGui::Separator();

        if (state.controllers.empty()) {
            ImGui::TextDisabled("Searching for controllers...");
        } else {
            float enableSz = ImGui::GetFrameHeight();
            ImDrawList* ctrlDrawList = ImGui::GetWindowDrawList();

            for (auto& entry : state.controllers) {
                ImGui::PushID(entry.id.c_str());

                // Status indicator square — same size as enable toggle
                {
                    ImVec2 p = ImGui::GetCursorScreenPos();
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

                    ImGui::InvisibleButton("##status", ImVec2(enableSz, enableSz));
                    if (ImGui::IsItemHovered()) {
                        ImGui::BeginTooltip();
                        if (!entry.controller || !entry.enabled) {
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
                toggleButton("##enable", &entry.enabled, entry.connecting);
                ImGui::SameLine();
                ImGui::Text("%s", entry.label.c_str());

                if (ImGui::IsItemHovered()) {
                    ImGui::BeginTooltip();
                    ImGui::Text("Type: %s", entry.type.c_str());
                    if (entry.maxPointRate > 0) ImGui::Text("Max: %u pps", entry.maxPointRate);
                    ImGui::Text("ID: %s", entry.id.c_str());
                    ImGui::EndTooltip();
                }

                if (entry.enabled && !wasEnabled && !entry.connecting)
                    startAsyncConnect(state, entry);
                else if (!entry.enabled && wasEnabled)
                    disconnectController(entry);

                ImGui::PopID();
            }
        }

        ImGui::EndChild();
        ImGui::End();

        ImGui::Render();
        int displayW, displayH;
        glfwGetFramebufferSize(window, &displayW, &displayH);
        glViewport(0, 0, displayW, displayH);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // Capture window geometry before saving
    glfwGetWindowSize(window, &state.windowWidth, &state.windowHeight);
    glfwGetWindowPos(window, &state.windowX, &state.windowY);
    saveSettings(state);

    state.discoveryRunning.store(false);
    if (state.discoveryThread.joinable()) state.discoveryThread.join();
    for (auto& entry : state.controllers) disconnectController(entry);
    state.liberaSystem.shutdown();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
