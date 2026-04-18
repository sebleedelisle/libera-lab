#include "LiberaPluginsWindow.h"

#include "LiberaFileDialog.h"
#include "fonts/IconsForkAwesome.h"
#include "imgui.h"
#include "libera/plugin/PluginRegistry.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#elif defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace libera::ui {

namespace {

namespace fs = std::filesystem;
using PluginInfo = libera::plugin::PluginInfo;
using PluginInstallResult = libera::plugin::PluginInstallResult;
using PluginRuntimeError = libera::plugin::PluginRuntimeError;
using PluginState = libera::plugin::PluginState;

struct UiState {
    std::string lastInstallMessage;
    bool lastInstallOk = false;
    std::string lastRemoveMessage;
    bool lastRemoveOk = false;
    bool restartHintVisible = false;
    std::unordered_set<std::string> pendingRestartPaths;
};

#ifdef _WIN32
constexpr const char* kPluginExtension = "dll";
#elif defined(__APPLE__)
constexpr const char* kPluginExtension = "dylib";
#else
constexpr const char* kPluginExtension = "so";
#endif

constexpr ImU32 kLoadedColor = IM_COL32(60, 200, 90, 255);
constexpr ImU32 kPendingColor = IM_COL32(230, 180, 70, 255);
constexpr ImU32 kFailureColor = IM_COL32(230, 110, 110, 255);
constexpr ImU32 kNeutralColor = IM_COL32(150, 150, 150, 255);

struct PluginStatusView {
    const char* label = "";
    ImU32 color = kNeutralColor;
};

struct PluginEntryView {
    std::string filename;
    std::string path;
    std::string displayName;
    std::string typeName;
    bool inUserPluginDir = false;
    bool pendingRestart = false;
    bool hasRegistryState = false;
    PluginState state = PluginState::FailedLoad;
    std::optional<std::string> loadError;
    std::vector<PluginRuntimeError> runtimeErrors;
};

struct InstallAttempt {
    PluginInstallResult result;
    std::string destinationPath;
    bool changedOnDisk = false;
};

UiState& uiState() {
    static UiState s;
    return s;
}

std::string normalizePathString(const fs::path& path) {
    std::error_code ec;
    const fs::path absolutePath = fs::absolute(path, ec);
    return (ec ? path : absolutePath).lexically_normal().string();
}

bool isSharedLibrary(const fs::path& path) {
    const std::string ext = path.extension().string();
    return ext == ".dylib" || ext == ".so" || ext == ".dll";
}

bool isInUserPluginDirectory(const std::string& pluginPath,
                             const std::string& userPluginDir) {
    return fs::path(normalizePathString(pluginPath)).parent_path() ==
           fs::path(normalizePathString(userPluginDir));
}

bool fileExists(const std::string& path) {
    std::error_code ec;
    return fs::is_regular_file(path, ec);
}

std::vector<fs::path> listSharedLibraries(const std::string& directory) {
    std::vector<fs::path> files;
    std::error_code ec;
    if (!fs::is_directory(directory, ec)) {
        return files;
    }

    for (const auto& entry : fs::directory_iterator(directory, ec)) {
        if (ec) {
            break;
        }
        if (!entry.is_regular_file(ec) || ec || !isSharedLibrary(entry.path())) {
            ec.clear();
            continue;
        }
        files.push_back(entry.path());
    }

    std::sort(files.begin(), files.end(),
              [](const fs::path& a, const fs::path& b) {
                  return a.filename().string() < b.filename().string();
              });
    return files;
}

std::string pluginHeading(const PluginEntryView& plugin) {
    if (!plugin.displayName.empty()) {
        return plugin.displayName;
    }
    const std::string filename =
        plugin.filename.empty() ? fs::path(plugin.path).filename().string()
                                : plugin.filename;
    const std::string stem = fs::path(filename).stem().string();
    return stem.empty() ? filename : stem;
}

PluginStatusView pluginStatusFor(const PluginEntryView& plugin) {
    if (plugin.pendingRestart || !plugin.hasRegistryState) {
        return {"Pending Restart", kPendingColor};
    }

    switch (plugin.state) {
        case PluginState::Loaded:
            return {"Loaded", kLoadedColor};
        case PluginState::NotAPlugin:
            return {"Not a Libera Plugin", kNeutralColor};
        case PluginState::FailedLoad:
            return {"Load Failed", kFailureColor};
        case PluginState::FailedValidation:
            return {"Validation Failed", kFailureColor};
        case PluginState::FailedBackend:
            return {"Backend Init Failed", kFailureColor};
    }
    return {"Unknown", kNeutralColor};
}

PluginEntryView makePluginEntry(const PluginInfo& info,
                                const UiState& state,
                                const std::string& userPluginDir) {
    PluginEntryView entry;
    entry.path = normalizePathString(info.path);
    entry.filename = info.filename.empty()
        ? fs::path(entry.path).filename().string()
        : info.filename;
    entry.displayName = info.displayName;
    entry.typeName = info.typeName;
    entry.inUserPluginDir = isInUserPluginDirectory(entry.path, userPluginDir);
    entry.pendingRestart = state.pendingRestartPaths.count(entry.path) != 0;
    entry.hasRegistryState = true;
    entry.state = info.state;
    entry.loadError = info.loadError;
    entry.runtimeErrors = info.runtimeErrors;
    return entry;
}

void collectPluginEntries(const std::string& userPluginDir,
                          const UiState& state,
                          std::vector<PluginEntryView>& userPlugins,
                          std::vector<PluginEntryView>& otherPlugins) {
    std::vector<PluginEntryView> allPlugins;
    std::unordered_map<std::string, std::size_t> indexByPath;

    for (const auto& info : libera::plugin::PluginRegistry::instance().snapshot()) {
        auto entry = makePluginEntry(info, state, userPluginDir);
        indexByPath[entry.path] = allPlugins.size();
        allPlugins.push_back(std::move(entry));
    }

    for (const auto& path : listSharedLibraries(userPluginDir)) {
        const std::string normalizedPath = normalizePathString(path);
        const auto existing = indexByPath.find(normalizedPath);
        if (existing != indexByPath.end()) {
            auto& entry = allPlugins[existing->second];
            entry.inUserPluginDir = true;
            if (state.pendingRestartPaths.count(normalizedPath) != 0) {
                entry.pendingRestart = true;
            }
            continue;
        }

        PluginEntryView entry;
        entry.filename = path.filename().string();
        entry.path = normalizedPath;
        entry.inUserPluginDir = true;
        entry.pendingRestart = true;
        indexByPath[entry.path] = allPlugins.size();
        allPlugins.push_back(std::move(entry));
    }

    std::sort(allPlugins.begin(), allPlugins.end(),
              [](const PluginEntryView& a, const PluginEntryView& b) {
                  const std::string aHeading = pluginHeading(a);
                  const std::string bHeading = pluginHeading(b);
                  if (aHeading != bHeading) {
                      return aHeading < bHeading;
                  }
                  return a.path < b.path;
              });

    for (auto& entry : allPlugins) {
        if (entry.inUserPluginDir) {
            userPlugins.push_back(std::move(entry));
        } else {
            otherPlugins.push_back(std::move(entry));
        }
    }
}

InstallAttempt installPluginIntoUserDirectory(const std::string& sourcePath,
                                             const std::string& userPluginDir) {
    InstallAttempt attempt;
    const std::string normalizedSource = normalizePathString(sourcePath);
    const std::string destinationPath = normalizePathString(
        fs::path(userPluginDir) / fs::path(normalizedSource).filename());

    attempt.destinationPath = destinationPath;
    if (normalizedSource == destinationPath) {
        attempt.result = libera::plugin::validatePluginFile(normalizedSource);
        if (attempt.result.success) {
            attempt.result.installedPath = destinationPath;
            attempt.result.message = "Plugin is already installed in the user plugin directory.";
        }
        return attempt;
    }

    attempt.result = libera::plugin::installPluginFile(normalizedSource, userPluginDir);
    if (attempt.result.success) {
        attempt.destinationPath = attempt.result.installedPath.empty()
            ? destinationPath
            : normalizePathString(attempt.result.installedPath);
        attempt.changedOnDisk = true;
    }
    return attempt;
}

std::string installMessageFor(const InstallAttempt& attempt) {
    if (!attempt.result.success) {
        return attempt.result.message;
    }

    if (!attempt.changedOnDisk) {
        return attempt.result.message;
    }

    std::string message =
        "Installed " + fs::path(attempt.destinationPath).filename().string();
    if (!attempt.result.message.empty()) {
        message += " (" + attempt.result.message + ")";
    }
    message += ". Restart to load it.";
    return message;
}

void drawStatusText(const PluginStatusView& status) {
    ImGui::PushStyleColor(ImGuiCol_Text, status.color);
    ImGui::TextUnformatted(status.label);
    ImGui::PopStyleColor();
}

void drawPluginList(const std::vector<PluginEntryView>& plugins,
                    UiState& state) {
    for (const auto& plugin : plugins) {
        ImGui::PushID(plugin.path.c_str());

        const PluginStatusView status = pluginStatusFor(plugin);
        ImVec2 dotPos = ImGui::GetCursorScreenPos();
        const float dotR = ImGui::GetTextLineHeight() * 0.45f;
        dotPos.x += dotR;
        dotPos.y += ImGui::GetTextLineHeight() * 0.5f;
        ImGui::GetWindowDrawList()->AddCircleFilled(dotPos, dotR, status.color);
        ImGui::Dummy(ImVec2(dotR * 2.0f + 6.0f, ImGui::GetTextLineHeight()));
        ImGui::SameLine();

        const std::string headerLabel = pluginHeading(plugin) + "##hdr_" + plugin.path;
        const float removeBtnW = 110.0f;
        const bool canRemove = plugin.inUserPluginDir && fileExists(plugin.path);
        const bool open = ImGui::CollapsingHeader(
            headerLabel.c_str(), canRemove ? ImGuiTreeNodeFlags_AllowOverlap
                                           : ImGuiTreeNodeFlags_None);

        if (canRemove) {
            ImGui::SameLine(ImGui::GetContentRegionMax().x - removeBtnW);
            if (ImGui::Button(ICON_FK_TRASH_O "  Remove", ImVec2(removeBtnW, 0))) {
                std::string error;
                if (libera::plugin::removePluginFile(plugin.path, &error)) {
                    state.lastRemoveOk = true;
                    state.lastRemoveMessage =
                        "Removed " + fs::path(plugin.path).filename().string() +
                        ". Restart to unload it.";
                    state.lastInstallMessage.clear();
                    state.pendingRestartPaths.erase(plugin.path);
                    state.restartHintVisible = true;
                } else {
                    state.lastRemoveOk = false;
                    state.lastRemoveMessage = "Remove failed: " + error;
                }
            }
        }

        if (open) {
            ImGui::Indent();

            ImGui::TextDisabled("State:");
            ImGui::SameLine();
            drawStatusText(status);

            if (!plugin.typeName.empty()) {
                ImGui::TextDisabled("Type:");
                ImGui::SameLine();
                ImGui::TextWrapped("%s", plugin.typeName.c_str());
            }

            ImGui::TextDisabled("File:");
            ImGui::SameLine();
            ImGui::TextWrapped("%s", plugin.path.c_str());

            ImGui::TextDisabled("Source:");
            ImGui::SameLine();
            ImGui::TextUnformatted(
                plugin.inUserPluginDir ? "User plugin directory" : "Startup search path");

            ImGui::TextDisabled("Startup:");
            ImGui::SameLine();
            if (plugin.pendingRestart || !plugin.hasRegistryState) {
                ImGui::TextWrapped("%s",
                                   "This file is on disk but will not be scanned again until restart.");
            } else if (plugin.state == PluginState::Loaded) {
                ImGui::TextWrapped("%s", "Loaded successfully at startup.");
            } else {
                ImGui::TextWrapped("%s", "Seen at startup, but not loaded.");
            }

            if (plugin.loadError && !plugin.loadError->empty()) {
                ImGui::TextDisabled("Detail:");
                ImGui::SameLine();
                ImGui::TextWrapped("%s", plugin.loadError->c_str());
            }

            if (!plugin.runtimeErrors.empty()) {
                const auto& latest = plugin.runtimeErrors.back();
                std::string runtimeDetail = latest.code;
                if (!latest.message.empty()) {
                    if (!runtimeDetail.empty()) {
                        runtimeDetail += " - ";
                    }
                    runtimeDetail += latest.message;
                }

                ImGui::TextDisabled("Runtime:");
                ImGui::SameLine();
                ImGui::TextWrapped("%s", runtimeDetail.c_str());

                if (plugin.runtimeErrors.size() > 1) {
                    ImGui::TextDisabled("History:");
                    ImGui::SameLine();
                    ImGui::Text("%zu runtime errors recorded this session",
                                plugin.runtimeErrors.size());
                }
            }

            ImGui::Unindent();
        }

        ImGui::PopID();
        ImGui::Spacing();
    }
}

void relaunchAndExit() {
#ifdef __APPLE__
    char buf[4096];
    uint32_t size = sizeof(buf);
    if (_NSGetExecutablePath(buf, &size) != 0) std::exit(0);
    // Walk up from .../Foo.app/Contents/MacOS/Foo to .../Foo.app
    std::string exe(buf);
    auto contentsMacOs = exe.rfind("/Contents/MacOS/");
    std::string target = (contentsMacOs != std::string::npos)
        ? exe.substr(0, contentsMacOs) : exe;
    std::string cmd = "(sleep 0.4; open -n \"" + target + "\") &";
    std::system(cmd.c_str());
    std::exit(0);
#elif defined(_WIN32)
    char path[MAX_PATH];
    GetModuleFileNameA(nullptr, path, MAX_PATH);
    STARTUPINFOA si{}; si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};
    CreateProcessA(path, nullptr, nullptr, nullptr, FALSE,
                   DETACHED_PROCESS, nullptr, nullptr, &si, &pi);
    if (pi.hProcess) { CloseHandle(pi.hProcess); CloseHandle(pi.hThread); }
    std::exit(0);
#else
    char buf[4096];
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) std::exit(0);
    buf[n] = '\0';
    std::string cmd = std::string("(sleep 0.4; \"") + buf + "\") &";
    std::system(cmd.c_str());
    std::exit(0);
#endif
}

} // namespace

void DrawPluginsWindow(bool* open, const std::string& userPluginDir) {
    if (!open || !*open) return;

    auto& s = uiState();
    std::vector<PluginEntryView> userPlugins;
    std::vector<PluginEntryView> otherPlugins;
    collectPluginEntries(userPluginDir, s, userPlugins, otherPlugins);

    ImGui::SetNextWindowSize(ImVec2(720, 520), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin(ICON_FK_PLUS_CIRCLE "  Plugins", open)) {
        ImGui::End();
        return;
    }

    ImGui::TextDisabled("User plugin directory:");
    ImGui::SameLine();
    ImGui::TextWrapped("%s", userPluginDir.c_str());
    ImGui::TextDisabled(
        "Plugins in this folder are loaded at startup. Install/remove changes require a restart.");
    ImGui::TextDisabled(
        "Startup plugin status below comes from Libera's plugin registry.");

    if (s.restartHintVisible) {
        if (ImGui::Button(ICON_FK_REFRESH "  Restart now", ImVec2(140, 0))) {
            relaunchAndExit();
        }
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(220, 180, 70, 255));
        ImGui::TextWrapped(ICON_FK_EXCLAMATION_TRIANGLE
                           "  Restart the app for plugin changes to take effect.");
        ImGui::PopStyleColor();
    }

    ImGui::Separator();

    // ---- Install button ----
    if (ImGui::Button(ICON_FK_UPLOAD "  Install new plugin...", ImVec2(220, 0))) {
        std::string picked = libera::ui::OpenFileDialog(
            "Choose a Libera plugin to install", {kPluginExtension});
        if (!picked.empty()) {
            const auto attempt = installPluginIntoUserDirectory(picked, userPluginDir);
            s.lastInstallOk = attempt.result.success;
            s.lastInstallMessage = installMessageFor(attempt);
            s.lastRemoveMessage.clear();
            if (attempt.result.success && attempt.changedOnDisk) {
                s.pendingRestartPaths.insert(attempt.destinationPath);
                s.restartHintVisible = true;
            }
        }
    }

    if (!s.lastInstallMessage.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text,
            s.lastInstallOk ? IM_COL32(120, 220, 120, 255)
                            : IM_COL32(230, 120, 120, 255));
        ImGui::TextWrapped("%s", s.lastInstallMessage.c_str());
        ImGui::PopStyleColor();
    }
    if (!s.lastRemoveMessage.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text,
            s.lastRemoveOk ? kPendingColor : kFailureColor);
        ImGui::TextWrapped("%s", s.lastRemoveMessage.c_str());
        ImGui::PopStyleColor();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::TextDisabled("User plugins");
    if (userPlugins.empty()) {
        ImGui::TextDisabled("No user-installed plugins found.");
    } else {
        drawPluginList(userPlugins, s);
    }

    if (!otherPlugins.empty()) {
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        ImGui::TextDisabled("Other plugins found at startup");
        drawPluginList(otherPlugins, s);
    }

    ImGui::End();
}

} // namespace libera::ui
