#pragma once

#include <cstdint>
#include <filesystem>

namespace gw {

// Writes a single-channel 8-bit grayscale image to disk as PNG. Returns true
// on success, false on any I/O or encoding failure. `data` points to the top
// row; consecutive rows are `stride_bytes` apart (use width for tightly-packed
// buffers, or CVPixelBufferGetBytesPerRow() for IOSurface-backed Mono8).
//
// Thin wrapper around libspng so the rest of the codebase doesn't drag spng.h
// into every TU.
bool write_mono8_png(const std::filesystem::path& path,
                     uint32_t                     width,
                     uint32_t                     height,
                     size_t                       stride_bytes,
                     const uint8_t*               data);

}  // namespace gw
