// LiberaPluginsWindow — ImGui panel for managing user-installed Libera plugins.
//
// Upstream Libera loads plugins directly from configured search paths. This
// window manages the user plugin directory on disk, and also shows startup load
// status reported by Libera's plugin registry. Install/remove changes still
// require a restart to take effect.

#pragma once

#include <string>

namespace libera::ui {

// Renders the Plugins window. Pass an externally-owned `open` bool that the
// caller toggles via a button or menu item; the window closes itself by
// flipping it to false. `userPluginDir` is the directory plugins are
// installed into / removed from (typically getSettingsDir()/plugins).
void DrawPluginsWindow(bool* open, const std::string& userPluginDir);

} // namespace libera::ui
