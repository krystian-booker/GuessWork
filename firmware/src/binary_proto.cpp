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

void BinRxParser::feed(uint8_t b) {
    switch (state_) {
        case State::Magic0:
            if (b == kBinMagic0) state_ = State::Magic1;
            break;
        case State::Magic1:
            state_ = (b == kBinMagic1) ? State::Type : State::Magic0;
            break;
        case State::Type:
            type_  = b;
            state_ = State::Len;
            break;
        case State::Len:
            if (b > kMaxPayload) {
                ++crc_errors_;
                state_ = State::Magic0;
                break;
            }
            len_   = b;
            got_   = 0;
            state_ = (len_ == 0) ? State::CrcLo : State::Payload;
            break;
        case State::Payload:
            buf_[got_++] = b;
            if (got_ == len_) state_ = State::CrcLo;
            break;
        case State::CrcLo:
            crc_lo_ = b;
            state_  = State::CrcHi;
            break;
        case State::CrcHi: {
            const uint16_t rx_crc = static_cast<uint16_t>(crc_lo_ | (b << 8));
            uint16_t crc = crc16_ccitt(&type_, 1);
            crc          = crc16_ccitt(&len_, 1, crc);
            crc          = crc16_ccitt(buf_, len_, crc);
            if (crc == rx_crc) {
                // Latest-wins latch: an unconsumed frame is overwritten
                // (the main loop drains far faster than the host sends).
                pend_type_ = type_;
                pend_len_  = len_;
                memcpy(pend_buf_, buf_, len_);
                pending_ = true;
            } else {
                ++crc_errors_;
            }
            state_ = State::Magic0;
            break;
        }
    }
}

bool BinRxParser::take(uint8_t& type, uint8_t* payload, uint8_t& len) {
    if (!pending_) return false;
    type = pend_type_;
    len  = pend_len_;
    memcpy(payload, pend_buf_, pend_len_);
    pending_ = false;
    return true;
}

}  // namespace gw_fw
