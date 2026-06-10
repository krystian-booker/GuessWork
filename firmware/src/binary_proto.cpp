#include "binary_proto.h"

namespace gw_fw {

uint16_t crc16_ccitt(const uint8_t* data, size_t n, uint16_t crc) {
    for (size_t i = 0; i < n; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

size_t build_frame(BinType type, const uint8_t* payload, uint8_t payload_len,
                   uint8_t* out) {
    size_t off = 0;
    out[off++] = kBinMagic0;
    out[off++] = kBinMagic1;
    out[off++] = static_cast<uint8_t>(type);
    out[off++] = payload_len;
    memcpy(out + off, payload, payload_len);
    off += payload_len;
    // CRC over type + len + payload (everything after the magic).
    const uint16_t crc = crc16_ccitt(out + 2, static_cast<size_t>(payload_len) + 2);
    out[off++] = static_cast<uint8_t>(crc);
    out[off++] = static_cast<uint8_t>(crc >> 8);
    return off;
}

}  // namespace gw_fw
