#include "LiberaFileDialog.h"

#import <AppKit/AppKit.h>

namespace libera::ui {

std::string OpenFileDialog(const char* title,
                           const std::vector<std::string>& extensions) {
    @autoreleasepool {
        NSOpenPanel* panel = [NSOpenPanel openPanel];
        if (title && *title) {
            [panel setMessage:[NSString stringWithUTF8String:title]];
        }
        [panel setCanChooseFiles:YES];
        [panel setCanChooseDirectories:NO];
        [panel setAllowsMultipleSelection:NO];
        [panel setResolvesAliases:YES];

        if (!extensions.empty()) {
            NSMutableArray<NSString*>* exts = [NSMutableArray array];
            for (const auto& e : extensions) {
                [exts addObject:[NSString stringWithUTF8String:e.c_str()]];
            }
            // Deprecated in 12.0 but still works; UTType API requires more setup.
            [panel setAllowedFileTypes:exts];
        }

        if ([panel runModal] != NSModalResponseOK) return {};
        NSURL* url = [[panel URLs] firstObject];
        if (!url) return {};
        return std::string([[url path] UTF8String]);
    }
}

} // namespace libera::ui
