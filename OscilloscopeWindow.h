// OscilloscopeWindow — ImGui popup showing XYRGB channel waveforms
// as they would appear on the DAC point stream.

#pragma once

#include "libera/core/LaserPoint.hpp"
#include <vector>

namespace libera::ui {

struct OscilloscopeState {
    float timeWindowMs = 0.0f;  // visible time span (0 = auto-fit to full frame)
    float offsetMs = 0.0f;      // scroll offset into the frame
};

// Renders the oscilloscope window. `open` controls visibility.
// `framePoints` is the current frame's point list; `pointRate` is
// the effective DAC rate so we can simulate the repeating stream.
// `flipX`, `flipY`, `orientation` are the output transform settings.
void DrawOscilloscopeWindow(bool* open,
                            OscilloscopeState& oscState,
                            const std::vector<core::LaserPoint>& framePoints,
                            int pointRate,
                            bool flipX, bool flipY, int orientation);

} // namespace libera::ui
