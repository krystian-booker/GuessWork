#include "app/preview_window.hpp"

#import <AppKit/AppKit.h>
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>

#include <string>
#include <utility>

namespace {
constexpr CGFloat kStatsAreaHeight = 56.0;
}

// -----------------------------------------------------------------------------
// Window delegate: bridges windowWillClose: back to a C++ std::function.
// -----------------------------------------------------------------------------
@interface GwWindowDelegate : NSObject <NSWindowDelegate>
@property(nonatomic, assign) std::function<void()>* handler;
@end

@implementation GwWindowDelegate
- (void)windowWillClose:(NSNotification*)__unused notification {
    if (_handler && *_handler) {
        (*_handler)();
    }
}
@end

namespace gw {

struct PreviewWindow::Impl {
    NSWindow*             window   = nil;
    NSView*               root     = nil;
    NSView*               metal_view = nil;
    CAMetalLayer*         layer    = nil;
    NSTextField*          stats    = nil;
    GwWindowDelegate*     delegate = nil;
    std::function<void()> close_handler;
};

PreviewWindow::PreviewWindow(uint32_t image_width, uint32_t image_height, std::string_view title)
    : impl_(std::make_unique<Impl>()) {
    const CGFloat W = image_width;
    const CGFloat H = image_height;
    const CGFloat content_w = W;
    const CGFloat content_h = H + kStatsAreaHeight;

    const NSRect frame = NSMakeRect(0, 0, content_w, content_h);
    const NSWindowStyleMask style =
        NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskMiniaturizable;

    impl_->window = [[NSWindow alloc] initWithContentRect:frame
                                                styleMask:style
                                                  backing:NSBackingStoreBuffered
                                                    defer:NO];
    NSString* nsTitle = [[NSString alloc] initWithBytes:title.data()
                                                  length:title.size()
                                                encoding:NSUTF8StringEncoding];
    [impl_->window setTitle:nsTitle];
    [impl_->window center];

    impl_->root = [[NSView alloc] initWithFrame:frame];

    // ----- Metal view occupies the top region (above the stats area) -----
    impl_->metal_view = [[NSView alloc] initWithFrame:NSMakeRect(0, kStatsAreaHeight, W, H)];
    impl_->metal_view.wantsLayer = YES;

    const CGFloat scale = [[NSScreen mainScreen] backingScaleFactor];
    impl_->layer                  = [CAMetalLayer layer];
    impl_->layer.device           = MTLCreateSystemDefaultDevice();
    impl_->layer.pixelFormat      = MTLPixelFormatBGRA8Unorm;
    impl_->layer.framebufferOnly  = YES;
    impl_->layer.contentsScale    = scale;
    impl_->layer.drawableSize     = CGSizeMake(W * scale, H * scale);
    impl_->layer.frame            = NSMakeRect(0, 0, W, H);   // local to metal_view
    impl_->metal_view.layer       = impl_->layer;
    [impl_->root addSubview:impl_->metal_view];

    // ----- Stats label below the image -----
    impl_->stats = [[NSTextField alloc]
        initWithFrame:NSMakeRect(8, 4, W - 16, kStatsAreaHeight - 8)];
    impl_->stats.bezeled        = NO;
    impl_->stats.drawsBackground = NO;
    impl_->stats.editable       = NO;
    impl_->stats.selectable     = NO;
    impl_->stats.font           = [NSFont monospacedSystemFontOfSize:11
                                                              weight:NSFontWeightRegular];
    impl_->stats.textColor      = [NSColor secondaryLabelColor];
    impl_->stats.usesSingleLineMode = NO;
    impl_->stats.lineBreakMode  = NSLineBreakByTruncatingTail;
    impl_->stats.cell.wraps     = YES;
    impl_->stats.stringValue    = @"warming up…";
    [impl_->root addSubview:impl_->stats];

    [impl_->window setContentView:impl_->root];

    impl_->delegate         = [[GwWindowDelegate alloc] init];
    impl_->delegate.handler = &impl_->close_handler;
    [impl_->window setDelegate:impl_->delegate];

    [impl_->window makeKeyAndOrderFront:nil];
}

PreviewWindow::~PreviewWindow() {
    if (impl_->window) {
        [impl_->window setDelegate:nil];
    }
}

void* PreviewWindow::metal_layer() const {
    return (__bridge void*)impl_->layer;
}

void PreviewWindow::set_close_handler(std::function<void()> handler) {
    impl_->close_handler = std::move(handler);
}

void PreviewWindow::set_stats_text(std::string_view text) {
    if (!impl_->stats) return;
    NSString* s = [[NSString alloc] initWithBytes:text.data()
                                           length:text.size()
                                         encoding:NSUTF8StringEncoding];
    impl_->stats.stringValue = s ?: @"";
}

}  // namespace gw
