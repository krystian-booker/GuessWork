#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string_view>

namespace gw {

// Thin C++ wrapper around an NSWindow + NSView backed by a CAMetalLayer.
//
// Must be constructed on the macOS main thread, before [NSApp run].
// Exposes the underlying CAMetalLayer as void* so non-ObjC headers can pass it
// around; the PreviewConsumer (which is .mm) bridges it back to CAMetalLayer*.
class PreviewWindow {
public:
    // image_width / image_height are the logical (point) size of the image
    // display area. The window adds a fixed stats area below it.
    PreviewWindow(uint32_t image_width, uint32_t image_height, std::string_view title);
    ~PreviewWindow();

    PreviewWindow(const PreviewWindow&)            = delete;
    PreviewWindow& operator=(const PreviewWindow&) = delete;

    // Returns CAMetalLayer* (as a type-erased pointer).
    void* metal_layer() const;

    // Called on the main thread when the user closes the window.
    void set_close_handler(std::function<void()> handler);

    // Replace the stats area's text. Must be called on the main thread.
    void set_stats_text(std::string_view text);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace gw
