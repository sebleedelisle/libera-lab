#include "LiberaSettingsWindow.h"

#include "fonts/IconsForkAwesome.h"
#include "imgui.h"
#include "libera/System.hpp"
#include "libera/plugin/PluginManagement.hpp"
#include "libera/plugin/PluginSettings.hpp"

#include <algorithm>

namespace libera::ui {
namespace {

const ImVec4 kErrorColor(1.0f, 0.35f, 0.35f, 1.0f);
const ImVec4 kSuccessColor(0.35f, 0.8f, 0.45f, 1.0f);

// Pick the driver System would activate for a controller type: the saved
// choice if it is still installed, otherwise the built-in, otherwise a sole
// plugin. Returns nullptr when several plugins compete and none was chosen.
const core::ControllerManagerInfo* activeDriver(
    const std::vector<core::ControllerManagerInfo>& implementations,
    const std::unordered_map<std::string, std::string>& selectedDrivers,
    const std::string& type) {
    const auto saved = selectedDrivers.find(type);
    if (saved != selectedDrivers.end()) {
        const auto found = std::find_if(
            implementations.begin(), implementations.end(),
            [&](const auto& implementation) {
                return implementation.driverId == saved->second;
            });
        if (found != implementations.end()) return &*found;
    }
    const auto builtIn = std::find_if(
        implementations.begin(), implementations.end(),
        [](const auto& implementation) { return implementation.builtIn; });
    if (builtIn != implementations.end()) return &*builtIn;
    if (implementations.size() == 1) return &implementations.front();
    return nullptr;
}

bool drawControllerDiscovery(std::set<std::string>& disabledControllerTypes) {
    bool changed = false;

    ImGui::SeparatorText("Controller Discovery");

    if (!disabledControllerTypes.empty()) {
        if (ImGui::Button(ICON_FK_CHECK "  Enable All")) {
            disabledControllerTypes.clear();
            changed = true;
        }
    }
    ImGui::Spacing();

    const auto availableManagers = libera::System::availableControllerManagers();
    if (availableManagers.empty()) {
        ImGui::TextDisabled("No controller managers registered.");
    }

    // Several managers can implement one controller type (a built-in plus
    // plugin drivers), so group them and offer a driver choice per type.
    std::unordered_map<std::string, std::vector<core::ControllerManagerInfo>>
        managersByType;
    std::vector<std::string> managerTypes;
    for (const auto& manager : availableManagers) {
        if (manager.type.empty()) continue;
        if (managersByType.find(manager.type) == managersByType.end()) {
            managerTypes.push_back(manager.type);
        }
        managersByType[manager.type].push_back(manager);
    }
    std::sort(managerTypes.begin(), managerTypes.end());
    const auto selectedDrivers = libera::plugin::selectedControllerDrivers();

    for (const auto& type : managerTypes) {
        const auto& implementations = managersByType[type];
        ImGui::PushID(type.c_str());

        bool enabled = disabledControllerTypes.find(type) ==
                       disabledControllerTypes.end();
        if (ImGui::Checkbox("##enabled", &enabled)) {
            if (enabled) disabledControllerTypes.erase(type);
            else disabledControllerTypes.insert(type);
            changed = true;
        }

        if (implementations.size() == 1) {
            const auto& implementation = implementations.front();
            const std::string label = implementation.displayName.empty()
                ? type
                : implementation.displayName;
            ImGui::SameLine();
            ImGui::TextUnformatted(label.c_str());
            if (!implementation.description.empty() && ImGui::IsItemHovered()) {
                ImGui::SetTooltip("%s", implementation.description.c_str());
            }
            if (label != type) {
                ImGui::SameLine();
                ImGui::TextDisabled("(%s)", type.c_str());
            }
            ImGui::PopID();
            continue;
        }

        ImGui::SameLine();
        ImGui::TextUnformatted(type.c_str());

        const auto* selected = activeDriver(implementations, selectedDrivers, type);
        ImGui::SameLine(180.0f);
        const std::string preview = selected ? selected->displayName
                                             : "Choose a driver";
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::BeginCombo("##driver", preview.c_str())) {
            for (const auto& implementation : implementations) {
                const bool isSelected =
                    selected && selected->driverId == implementation.driverId;
                const std::string label = implementation.displayName +
                    (implementation.builtIn ? " (built-in)" : " (plugin)");
                if (ImGui::Selectable(label.c_str(), isSelected) && !isSelected) {
                    if (libera::plugin::selectControllerDriver(
                            type, implementation.driverId)) {
                        changed = true;
                    }
                }
                if (ImGui::IsItemHovered() && !implementation.description.empty()) {
                    ImGui::SetTooltip("%s", implementation.description.c_str());
                }
            }
            ImGui::EndCombo();
        }

        ImGui::PopID();
    }

    return changed;
}

// Draw the plugin-provided settings of one controller plus the result of the
// last change.
void drawControllerPluginSettings(const std::string& driverId,
                                  const std::string& controllerId,
                                  libera::gui::imgui::PluginPanelState& state) {
    libera::gui::imgui::DrawPluginControllerSettings(driverId, controllerId, state);

    // Keep validation and plugin errors beside the setting that produced them
    // so they are immediately actionable.
    if (!state.lastMessage.empty()) {
        ImGui::TextColored(state.lastMessageIsError ? kErrorColor : kSuccessColor,
                           "%s",
                           state.lastMessage.c_str());
    }
}

void drawControllerSettings(const std::vector<SettingsControllerItem>& controllers,
                            SettingsWindowState& state) {
    // The section only appears when a discovered controller has settings.
    bool any = false;
    for (const auto& controller : controllers) {
        if (libera::plugin::controllerSettings(controller.driverId, controller.id)
                .empty()) {
            continue;
        }
        if (!any) {
            ImGui::Spacing();
            ImGui::SeparatorText("Plugin Controller Settings");
            any = true;
        }

        const std::string settingsKey = controller.driverId + "\n" + controller.id;
        ImGui::PushID(settingsKey.c_str());
        const std::string label = controller.label + " (" + controller.type + ")";
        if (ImGui::TreeNodeEx(label.c_str(), ImGuiTreeNodeFlags_SpanAvailWidth)) {
            drawControllerPluginSettings(
                controller.driverId,
                controller.id,
                state.controllerSettingsStates[settingsKey]);
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
}

} // namespace

bool DrawSettingsWindow(bool* open,
                        std::set<std::string>& disabledControllerTypes,
                        const std::vector<SettingsControllerItem>& controllers,
                        SettingsWindowState& state) {
    if (!open || !*open) {
        return false;
    }

    bool changed = false;
    ImGui::SetNextWindowSize(ImVec2(620.0f, 500.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(ICON_FK_COG "  Settings", open, ImGuiWindowFlags_NoCollapse)) {
        changed = drawControllerDiscovery(disabledControllerTypes);
        drawControllerSettings(controllers, state);
    }
    ImGui::End();
    return changed;
}

} // namespace libera::ui
