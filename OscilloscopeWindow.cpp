#include "OscilloscopeWindow.h"

#include "fonts/IconsForkAwesome.h"
#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace libera::ui {

// Apply the same output transform that main.cpp applies before sending
static void applyTransform(float& x, float& y, bool flipX, bool flipY, int orientation) {
    if (flipX) x = -x;
    if (flipY) y = -y;
    for (int r = 0; r < orientation; ++r) {
        float tmp = x;
        x = -y;
        y = tmp;
    }
}

void DrawOscilloscopeWindow(bool* open,
                            OscilloscopeState& oscState,
                            const std::vector<core::LaserPoint>& framePoints,
                            int pointRate,
                            bool flipX, bool flipY, int orientation)
{
    if (!open || !*open) return;

    ImGui::SetNextWindowSize(ImVec2(600, 480), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin(ICON_FK_SIGNAL "  Oscilloscope", open)) {
        ImGui::End();
        return;
    }

    if (framePoints.empty() || pointRate <= 0) {
        ImGui::TextDisabled("No frame data");
        ImGui::End();
        return;
    }

    int numFramePts = static_cast<int>(framePoints.size());
    float frameDurationMs = (static_cast<float>(numFramePts) / pointRate) * 1000.0f;

    // Auto-fit to full frame on first use
    if (oscState.timeWindowMs <= 0.0f)
        oscState.timeWindowMs = frameDurationMs;

    // Controls
    ImGui::SliderFloat("Zoom (ms)", &oscState.timeWindowMs, 0.5f,
                       std::max(frameDurationMs, 1.0f), "%.1f ms");

    // Clamp offset so the visible window stays within [0, frameDuration]
    float maxOffset = std::max(0.0f, frameDurationMs - oscState.timeWindowMs);
    if (oscState.offsetMs > maxOffset) oscState.offsetMs = maxOffset;
    bool offsetUseful = maxOffset > 0.01f;
    if (!offsetUseful) { oscState.offsetMs = 0.0f; ImGui::BeginDisabled(); }
    ImGui::SliderFloat("Offset (ms)", &oscState.offsetMs, 0.0f,
                       std::max(maxOffset, 0.001f), "%.2f ms");
    if (!offsetUseful) ImGui::EndDisabled();

    ImGui::Text("%d points  %.2f ms @ %d pps", numFramePts, frameDurationMs, pointRate);

    // How many samples are in the visible window
    int windowSamples = static_cast<int>(oscState.timeWindowMs * 0.001f * pointRate);
    windowSamples = std::clamp(windowSamples, 1, numFramePts);

    // Starting sample from offset — clamp so start + window doesn't exceed frame
    int startSample = static_cast<int>(oscState.offsetMs * 0.001f * pointRate);
    startSample = std::clamp(startSample, 0, numFramePts - windowSamples);

    // Channel info — RGB colours match the HSV(h, 0.6, 0.6) used for the main UI buttons
    struct Channel {
        const char* label;
        ImU32 colour;
        ImU32 fillColour;
        float minVal, maxVal;
        bool filled;
    };
    Channel channels[5] = {
        {"X", IM_COL32(255, 255, 255, 255), 0,                          -1.0f, 1.0f, false},
        {"Y", IM_COL32(255, 255, 255, 255), 0,                          -1.0f, 1.0f, false},
        {"R", IM_COL32(153, 61,  61,  255), IM_COL32(153, 61,  61,  80), 0.0f, 1.0f, true},
        {"G", IM_COL32(61,  153, 61,  255), IM_COL32(61,  153, 61,  80), 0.0f, 1.0f, true},
        {"B", IM_COL32(61,  61,  153, 255), IM_COL32(61,  61,  153, 80), 0.0f, 1.0f, true},
    };

    float availW = ImGui::GetContentRegionAvail().x;
    float availH = ImGui::GetContentRegionAvail().y;
    float labelW = 24.0f;
    float plotW = availW - labelW - 4.0f;
    float channelH = (availH - 4.0f * 4.0f) / 5.0f; // 5 channels, 4 gaps
    channelH = std::max(channelH, 30.0f);

    ImDrawList* drawList = ImGui::GetWindowDrawList();
    ImU32 bgCol = IM_COL32(20, 20, 20, 255);
    ImU32 gridCol = IM_COL32(50, 50, 50, 255);
    ImU32 zeroCol = IM_COL32(80, 80, 80, 255);

    for (int ch = 0; ch < 5; ++ch) {
        ImVec2 cursor = ImGui::GetCursorScreenPos();

        // Label
        ImGui::SetCursorScreenPos(cursor);
        ImGui::TextColored(ImColor(channels[ch].colour).Value, "%s", channels[ch].label);

        // Plot area
        ImVec2 plotMin(cursor.x + labelW, cursor.y);
        ImVec2 plotMax(plotMin.x + plotW, plotMin.y + channelH);

        // Background
        drawList->AddRectFilled(plotMin, plotMax, bgCol);

        // Grid lines (horizontal: 25%, 50%, 75%)
        for (int g = 1; g <= 3; ++g) {
            float gy = plotMin.y + channelH * (g / 4.0f);
            drawList->AddLine(ImVec2(plotMin.x, gy), ImVec2(plotMax.x, gy), gridCol);
        }

        // Zero/centre line
        float minV = channels[ch].minVal;
        float maxV = channels[ch].maxVal;
        float zeroNorm = (0.0f - minV) / (maxV - minV);
        float zeroY = plotMax.y - zeroNorm * channelH;
        if (zeroY > plotMin.y && zeroY < plotMax.y)
            drawList->AddLine(ImVec2(plotMin.x, zeroY), ImVec2(plotMax.x, zeroY), zeroCol);

        // Clip waveform drawing to plot area
        drawList->PushClipRect(plotMin, plotMax, true);

        // Draw waveform
        int drawPoints = std::min(windowSamples, static_cast<int>(plotW));
        float samplesPerPixel = static_cast<float>(windowSamples) / static_cast<float>(drawPoints);

        // Build screen positions for the waveform
        struct WavePt { float sx, sy; };
        std::vector<WavePt> wave(drawPoints);
        for (int i = 0; i < drawPoints; ++i) {
            int sampleIdx = startSample + static_cast<int>(i * samplesPerPixel);
            sampleIdx = std::clamp(sampleIdx, 0, numFramePts - 1);
            const auto& pt = framePoints[sampleIdx];

            float val;
            float px = pt.x, py = pt.y;
            applyTransform(px, py, flipX, flipY, orientation);
            switch (ch) {
                case 0: val = px;   break;
                case 1: val = py;   break;
                case 2: val = pt.r; break;
                case 3: val = pt.g; break;
                case 4: val = pt.b; break;
                default: val = 0.0f;
            }

            float norm = (val - minV) / (maxV - minV);
            norm = std::clamp(norm, 0.0f, 1.0f);
            wave[i].sx = plotMin.x + (static_cast<float>(i) / static_cast<float>(drawPoints - 1)) * plotW;
            wave[i].sy = plotMax.y - norm * channelH;
        }

        // Filled area for RGB channels (from baseline to waveform)
        if (channels[ch].filled && drawPoints > 1) {
            for (int i = 0; i < drawPoints - 1; ++i) {
                drawList->AddQuadFilled(
                    ImVec2(wave[i].sx,     wave[i].sy),
                    ImVec2(wave[i + 1].sx, wave[i + 1].sy),
                    ImVec2(wave[i + 1].sx, plotMax.y),
                    ImVec2(wave[i].sx,     plotMax.y),
                    channels[ch].fillColour);
            }
        }

        // Line on top
        for (int i = 1; i < drawPoints; ++i) {
            drawList->AddLine(ImVec2(wave[i - 1].sx, wave[i - 1].sy),
                              ImVec2(wave[i].sx,     wave[i].sy),
                              channels[ch].colour, 1.5f);
        }

        drawList->PopClipRect();

        // Border
        drawList->AddRect(plotMin, plotMax, IM_COL32(80, 80, 80, 255));

        ImGui::SetCursorScreenPos(ImVec2(cursor.x, plotMax.y + 4.0f));
    }

    ImGui::End();
}

} // namespace libera::ui
