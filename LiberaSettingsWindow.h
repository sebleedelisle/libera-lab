// LiberaSettingsWindow — controller discovery options and per-controller
// plugin settings, mirroring the Settings window in Libera Link.

#pragma once

#include "libera/gui/imgui/PluginManagementPanel.hpp"

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace libera::ui {

// One discovered controller, as far as the settings window is concerned.
struct SettingsControllerItem {
    std::string driverId;
    std::string id;
    std::string label;
    std::string type;
};

struct SettingsWindowState {
    // Draft/edit state for plugin controller settings, keyed by driver + id.
    std::unordered_map<std::string, libera::gui::imgui::PluginPanelState>
        controllerSettingsStates;
};

// Returns true when the discovery configuration changed (a controller type
// was enabled/disabled or a different driver was selected). The caller must
// then rebuild its libera::System, because managers are chosen at
// construction time.
bool DrawSettingsWindow(bool* open,
                        std::set<std::string>& disabledControllerTypes,
                        const std::vector<SettingsControllerItem>& controllers,
                        SettingsWindowState& state);

} // namespace libera::ui
