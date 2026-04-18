#include "LiberaFileDialog.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>
#include <sstream>
#elif !defined(__APPLE__)
#include <cstdio>
#include <cstdlib>
#include <sstream>
#endif

namespace libera::ui {

#ifdef _WIN32
std::string OpenFileDialog(const char* title,
                           const std::vector<std::string>& extensions) {
    char file[MAX_PATH] = {0};

    std::string filter;
    if (!extensions.empty()) {
        std::ostringstream label;
        label << "Allowed files (";
        for (size_t i = 0; i < extensions.size(); ++i) {
            if (i) label << ";";
            label << "*." << extensions[i];
        }
        label << ")";
        filter = label.str();
        filter.push_back('\0');
        for (size_t i = 0; i < extensions.size(); ++i) {
            if (i) filter += ";";
            filter += "*." + extensions[i];
        }
        filter.push_back('\0');
        filter += "All files\0*.*\0";
    } else {
        filter = std::string("All files\0*.*\0", 14);
    }

    OPENFILENAMEA ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFile   = file;
    ofn.nMaxFile    = sizeof(file);
    ofn.lpstrFilter = filter.c_str();
    ofn.nFilterIndex = 1;
    ofn.lpstrTitle  = title;
    ofn.Flags       = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
    if (!GetOpenFileNameA(&ofn)) return {};
    return file;
}
#elif !defined(__APPLE__)
// Linux: best-effort via zenity if available.
std::string OpenFileDialog(const char* title,
                           const std::vector<std::string>& extensions) {
    std::ostringstream cmd;
    cmd << "zenity --file-selection";
    if (title && *title) cmd << " --title=\"" << title << "\"";
    if (!extensions.empty()) {
        cmd << " --file-filter=\"";
        for (size_t i = 0; i < extensions.size(); ++i) {
            if (i) cmd << " ";
            cmd << "*." << extensions[i];
        }
        cmd << "\"";
    }
    cmd << " 2>/dev/null";
    FILE* p = popen(cmd.str().c_str(), "r");
    if (!p) return {};
    std::string out;
    char buf[1024];
    while (fgets(buf, sizeof(buf), p)) out += buf;
    pclose(p);
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
    return out;
}
#endif

} // namespace libera::ui
