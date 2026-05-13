#import <AppKit/AppKit.h>
#import <CoreFoundation/CoreFoundation.h>

#include <algorithm>
#include <cstdio>
#include <exception>
#include <iostream>

#include "app/preview_window.hpp"
#include "consumer/preview_consumer.hpp"
#include "producer/spinnaker_producer.hpp"
#include "producer/spinnaker_producer_internal.hpp"

namespace {

// Scale the camera's native resolution so the image area fits comfortably in
// the visible portion of the user's main display.
struct WindowSize { uint32_t w; uint32_t h; };

WindowSize compute_window_size(uint32_t cam_w, uint32_t cam_h) {
    const CGRect visible = [[NSScreen mainScreen] visibleFrame];
    const double max_w   = visible.size.width  * 0.7;
    const double max_h   = visible.size.height * 0.7;
    const double scale   = std::min({1.0, max_w / cam_w, max_h / cam_h});
    return WindowSize{
        .w = static_cast<uint32_t>(cam_w * scale),
        .h = static_cast<uint32_t>(cam_h * scale),
    };
}

}  // namespace

int main() {
    @autoreleasepool {
        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];

        // Debug binary — grab whichever Spinnaker camera is plugged in first.
        Spinnaker::SystemPtr  system = Spinnaker::System::GetInstance();
        Spinnaker::CameraList cams   = system->GetCameras();
        if (cams.GetSize() == 0) {
            std::cerr << "No Spinnaker cameras detected.\n";
            cams.Clear();
            system->ReleaseInstance();
            return 1;
        }
        Spinnaker::CameraPtr cam = cams.GetByIndex(0);
        cams.Clear();

        auto binding    = std::make_unique<gw::SpinnakerCameraBinding>();
        binding->system = system;
        binding->cam    = cam;

        gw::SpinnakerProducer producer{"cam0", "preview"};
        producer.bind_camera(std::move(binding));
        try {
            producer.start();
        } catch (const std::exception& e) {
            std::cerr << "Producer start failed: " << e.what() << "\n";
            return 1;
        }

        const auto fmt = producer.format();
        const auto sz  = compute_window_size(fmt.width, fmt.height);
        gw::PreviewWindow window{sz.w, sz.h, "GuessWork"};
        gw::PreviewConsumer consumer{"preview", window.metal_layer()};
        consumer.attach(producer.channel());

        // ---- Stats refresh timer ----
        auto* p_producer = &producer;
        auto* p_consumer = &consumer;
        auto* p_window   = &window;
        const uint32_t cam_w = fmt.width;
        const uint32_t cam_h = fmt.height;

        __block uint64_t        prev_pub  = 0;
        __block uint64_t        prev_recv = 0;
        __block CFAbsoluteTime  prev_t    = CFAbsoluteTimeGetCurrent();

        NSTimer* stats_timer = [NSTimer
            scheduledTimerWithTimeInterval:0.25
                                   repeats:YES
                                     block:^(NSTimer* __unused t) {
            const auto ps = p_producer->stats();
            const auto cs = p_consumer->stats();
            const CFAbsoluteTime now = CFAbsoluteTimeGetCurrent();
            const double dt = std::max(now - prev_t, 1e-6);

            const double pub_fps  = (ps.total_published - prev_pub)  / dt;
            const double recv_fps = (cs.total_received  - prev_recv) / dt;
            const double latency_ms = cs.last_latency_ns / 1e6;

            char line1[256];
            char line2[256];
            std::snprintf(line1, sizeof line1,
                          "cam %ux%u  producer %.1f fps  published %llu  dropped %llu  incomplete %llu",
                          cam_w, cam_h, pub_fps,
                          (unsigned long long)ps.total_published,
                          (unsigned long long)ps.total_dropped,
                          (unsigned long long)ps.total_incomplete);
            std::snprintf(line2, sizeof line2,
                          "preview  %.1f fps  received %llu  skipped %llu  seq %llu  latency %.1f ms",
                          recv_fps,
                          (unsigned long long)cs.total_received,
                          (unsigned long long)cs.total_dropped,
                          (unsigned long long)cs.last_sequence,
                          latency_ms);
            std::string s = line1;
            s += '\n';
            s += line2;
            p_window->set_stats_text(s);

            prev_pub  = ps.total_published;
            prev_recv = cs.total_received;
            prev_t    = now;
        }];
        // Run during modal panels too (e.g. close-confirm dialogs).
        [[NSRunLoop mainRunLoop] addTimer:stats_timer forMode:NSRunLoopCommonModes];

        window.set_close_handler([&] {
            [stats_timer invalidate];
            consumer.detach();
            producer.stop();
            [NSApp stop:nil];
            NSEvent* wakeup = [NSEvent otherEventWithType:NSEventTypeApplicationDefined
                                                 location:NSZeroPoint
                                            modifierFlags:0
                                                timestamp:0
                                             windowNumber:0
                                                  context:nil
                                                  subtype:0
                                                    data1:0
                                                    data2:0];
            [NSApp postEvent:wakeup atStart:YES];
        });

        [NSApp activateIgnoringOtherApps:YES];
        [NSApp run];
    }
    return 0;
}
