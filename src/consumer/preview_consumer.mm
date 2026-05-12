#include "consumer/preview_consumer.hpp"

#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>
#include <CoreVideo/CoreVideo.h>

#include <atomic>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "core/clock.hpp"
#include "core/frame.hpp"

namespace gw {

namespace {

// Single-file Metal shader: fullscreen triangle + grayscale-from-R sampling.
constexpr const char* kShaderSource = R"SHADER(
#include <metal_stdlib>
using namespace metal;

struct VOut {
    float4 pos [[position]];
    float2 uv;
};

vertex VOut vs_main(uint vid [[vertex_id]]) {
    // Single oversized triangle covering the viewport (clipped to the screen).
    const float2 positions[3] = { float2(-1, -1), float2( 3, -1), float2(-1,  3) };
    // UVs chosen so the bottom-left of the screen samples the top-left of the
    // image (CoreVideo / IOSurface textures are top-origin in Metal sampling).
    const float2 uvs[3]       = { float2( 0,  1), float2( 2,  1), float2( 0, -1) };
    VOut out;
    out.pos = float4(positions[vid], 0.0, 1.0);
    out.uv  = uvs[vid];
    return out;
}

fragment float4 fs_main(VOut in [[stage_in]],
                        texture2d<float> tex [[texture(0)]],
                        sampler           sam [[sampler(0)]]) {
    float r = tex.sample(sam, in.uv).r;
    return float4(r, r, r, 1.0);
}
)SHADER";

}  // namespace

// -----------------------------------------------------------------------------
// Impl
// -----------------------------------------------------------------------------

struct PreviewConsumer::Impl {
    std::string                       name;
    CAMetalLayer*                     layer = nil;

    id<MTLDevice>                     device = nil;
    id<MTLCommandQueue>               queue  = nil;
    id<MTLRenderPipelineState>        pipeline = nil;
    id<MTLSamplerState>               sampler  = nil;
    CVMetalTextureCacheRef            tex_cache = nullptr;

    FrameChannel*                     channel = nullptr;
    FrameChannel::SubscriberHandle    sub;

    std::atomic<bool>                 running{false};
    std::thread                       worker;

    std::atomic<uint64_t>             total_received{0};
    std::atomic<uint64_t>             total_dropped{0};
    std::atomic<uint64_t>             last_sequence{0};
    std::atomic<uint64_t>             last_latency_ns{0};

    void build_metal_pipeline();
    void render_loop();
    bool render_frame(Frame* f);
};

void PreviewConsumer::Impl::build_metal_pipeline() {
    device = layer.device ?: MTLCreateSystemDefaultDevice();
    if (!device) throw std::runtime_error("PreviewConsumer: no Metal device available");
    layer.device = device;

    queue = [device newCommandQueue];

    NSError* err = nil;
    id<MTLLibrary> lib =
        [device newLibraryWithSource:[NSString stringWithUTF8String:kShaderSource]
                             options:nil
                               error:&err];
    if (!lib) {
        throw std::runtime_error(std::string("Metal shader compile failed: ") +
                                 [[err localizedDescription] UTF8String]);
    }

    id<MTLFunction> vs = [lib newFunctionWithName:@"vs_main"];
    id<MTLFunction> fs = [lib newFunctionWithName:@"fs_main"];

    MTLRenderPipelineDescriptor* desc = [[MTLRenderPipelineDescriptor alloc] init];
    desc.vertexFunction              = vs;
    desc.fragmentFunction            = fs;
    desc.colorAttachments[0].pixelFormat = layer.pixelFormat;
    pipeline = [device newRenderPipelineStateWithDescriptor:desc error:&err];
    if (!pipeline) {
        throw std::runtime_error(std::string("Metal pipeline state failed: ") +
                                 [[err localizedDescription] UTF8String]);
    }

    MTLSamplerDescriptor* samp = [[MTLSamplerDescriptor alloc] init];
    samp.minFilter   = MTLSamplerMinMagFilterLinear;
    samp.magFilter   = MTLSamplerMinMagFilterLinear;
    samp.sAddressMode = MTLSamplerAddressModeClampToEdge;
    samp.tAddressMode = MTLSamplerAddressModeClampToEdge;
    sampler = [device newSamplerStateWithDescriptor:samp];

    if (CVMetalTextureCacheCreate(kCFAllocatorDefault, nullptr, device, nullptr, &tex_cache) !=
        kCVReturnSuccess) {
        throw std::runtime_error("CVMetalTextureCacheCreate failed");
    }
}

bool PreviewConsumer::Impl::render_frame(Frame* f) {
    CVPixelBufferRef pb = f->pixel_buffer();
    const size_t     w  = CVPixelBufferGetWidth(pb);
    const size_t     h  = CVPixelBufferGetHeight(pb);

    CVMetalTextureRef tex_ref = nullptr;
    const CVReturn    r       = CVMetalTextureCacheCreateTextureFromImage(
        kCFAllocatorDefault,
        tex_cache,
        pb,
        nullptr,
        MTLPixelFormatR8Unorm,
        w,
        h,
        /*planeIndex=*/0,
        &tex_ref);
    if (r != kCVReturnSuccess || !tex_ref) {
        if (tex_ref) CFRelease(tex_ref);
        return false;
    }
    id<MTLTexture> source_tex = CVMetalTextureGetTexture(tex_ref);

    @autoreleasepool {
        id<CAMetalDrawable> drawable = [layer nextDrawable];
        if (!drawable) {
            CFRelease(tex_ref);
            return false;
        }

        MTLRenderPassDescriptor* rp     = [MTLRenderPassDescriptor renderPassDescriptor];
        rp.colorAttachments[0].texture  = drawable.texture;
        rp.colorAttachments[0].loadAction  = MTLLoadActionClear;
        rp.colorAttachments[0].storeAction = MTLStoreActionStore;
        rp.colorAttachments[0].clearColor  = MTLClearColorMake(0, 0, 0, 1);

        id<MTLCommandBuffer>         cb  = [queue commandBuffer];
        id<MTLRenderCommandEncoder>  enc = [cb renderCommandEncoderWithDescriptor:rp];
        [enc setRenderPipelineState:pipeline];
        [enc setFragmentTexture:source_tex atIndex:0];
        [enc setFragmentSamplerState:sampler atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        [enc endEncoding];

        [cb presentDrawable:drawable];
        [cb commit];
    }

    CFRelease(tex_ref);
    return true;
}

void PreviewConsumer::Impl::render_loop() {
    while (running.load(std::memory_order_acquire)) {
        Frame* f = channel->next_frame(sub);
        if (!f) break;

        // Stats accounting (before render, in case render throws).
        const uint64_t seq      = f->sequence();
        const uint64_t prev_seq = last_sequence.load(std::memory_order_relaxed);
        if (prev_seq != 0 && seq > prev_seq + 1) {
            total_dropped.fetch_add(seq - prev_seq - 1, std::memory_order_relaxed);
        }
        last_sequence.store(seq, std::memory_order_relaxed);
        last_latency_ns.store(Clock::now_ns() - f->host_capture_ns(),
                              std::memory_order_relaxed);
        total_received.fetch_add(1, std::memory_order_relaxed);

        try {
            render_frame(f);
        } catch (const std::exception& e) {
            std::cerr << "PreviewConsumer render error: " << e.what() << "\n";
        }
        f->release();

        // Flush the texture cache periodically so stale textures don't pile up.
        CVMetalTextureCacheFlush(tex_cache, 0);
    }
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

PreviewConsumer::PreviewConsumer(std::string name, void* metal_layer)
    : impl_(std::make_unique<Impl>()) {
    impl_->name  = std::move(name);
    impl_->layer = (__bridge CAMetalLayer*)metal_layer;
    if (!impl_->layer) throw std::invalid_argument("PreviewConsumer: layer is null");
    impl_->build_metal_pipeline();
}

PreviewConsumer::~PreviewConsumer() {
    try {
        detach();
    } catch (...) {
    }
    if (impl_->tex_cache) {
        CFRelease(impl_->tex_cache);
        impl_->tex_cache = nullptr;
    }
}

std::string_view PreviewConsumer::name() const { return impl_->name; }

PreviewConsumerStats PreviewConsumer::stats() const {
    return PreviewConsumerStats{
        .total_received  = impl_->total_received.load(std::memory_order_relaxed),
        .total_dropped   = impl_->total_dropped.load(std::memory_order_relaxed),
        .last_sequence   = impl_->last_sequence.load(std::memory_order_relaxed),
        .last_latency_ns = impl_->last_latency_ns.load(std::memory_order_relaxed),
    };
}

void PreviewConsumer::attach(FrameChannel& ch) {
    if (impl_->running.load()) return;
    impl_->channel = &ch;
    impl_->sub     = ch.subscribe();
    impl_->running.store(true);
    impl_->worker = std::thread([this] { impl_->render_loop(); });
}

void PreviewConsumer::detach() {
    if (!impl_->running.exchange(false)) return;
    if (impl_->channel && impl_->sub) {
        impl_->channel->unsubscribe(impl_->sub);
    }
    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }
    impl_->sub.reset();
    impl_->channel = nullptr;
}

}  // namespace gw
