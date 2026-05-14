#include "consumer/recording_png.hpp"

#include <spng.h>

#include <cstdio>
#include <iostream>

namespace gw {

namespace {

// RAII guards keep error paths short.
struct FileGuard {
    FILE* fp = nullptr;
    ~FileGuard() { if (fp) std::fclose(fp); }
};
struct CtxGuard {
    spng_ctx* ctx = nullptr;
    ~CtxGuard() { if (ctx) spng_ctx_free(ctx); }
};

}  // namespace

bool write_mono8_png(const std::filesystem::path& path,
                     uint32_t                     width,
                     uint32_t                     height,
                     size_t                       stride_bytes,
                     const uint8_t*               data) {
    if (!data || width == 0 || height == 0 || stride_bytes < width) return false;

    FileGuard fg;
    fg.fp = std::fopen(path.string().c_str(), "wb");
    if (!fg.fp) return false;

    CtxGuard cg;
    cg.ctx = spng_ctx_new(SPNG_CTX_ENCODER);
    if (!cg.ctx) return false;

    if (spng_set_png_file(cg.ctx, fg.fp) != 0) return false;

    spng_ihdr ihdr{};
    ihdr.width      = width;
    ihdr.height     = height;
    ihdr.bit_depth  = 8;
    ihdr.color_type = SPNG_COLOR_TYPE_GRAYSCALE;
    if (spng_set_ihdr(cg.ctx, &ihdr) != 0) return false;

    // Progressive row encoding handles padded source buffers (stride > width)
    // without an intermediate tight copy: we hand libspng one row pointer at
    // a time.
    int rc = spng_encode_image(cg.ctx, nullptr, 0, SPNG_FMT_PNG,
                               SPNG_ENCODE_PROGRESSIVE | SPNG_ENCODE_FINALIZE);
    if (rc != 0) {
        std::cerr << "spng_encode_image init: " << spng_strerror(rc) << "\n";
        return false;
    }

    for (uint32_t y = 0; y < height; ++y) {
        rc = spng_encode_row(cg.ctx, data + y * stride_bytes, width);
        if (rc == SPNG_EOI) break;       // last row written successfully
        if (rc != 0) {
            std::cerr << "spng_encode_row[" << y << "]: " << spng_strerror(rc) << "\n";
            return false;
        }
    }
    return true;
}

}  // namespace gw
