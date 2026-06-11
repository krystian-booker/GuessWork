#include "can_bridge.h"

#include <FlexCAN_T4.h>

#include "binary_proto.h"
#include "time64.h"

// This must remain the ONLY translation unit that includes FlexCAN_T4.h: the
// library's ISR dispatch pointer (`_CAN3`) is a per-TU header static, and we
// reassign it on mode switches (see set_mode below).
//
// NEVER call events() on either object. While events() is unused, onReceive
// callbacks fire directly in ISR context (FlexCAN_T4.tpp, struct2queueRx),
// which is what gives our arrival stamps their precision. One events() call
// permanently defers callbacks to loop context. TX likewise sticks to the
// direct-mailbox write() path.

namespace gw_fw {

namespace {

// --- controller objects (both constructed; at most one begun at a time) ----
FlexCAN_T4<CAN3, RX_SIZE_64, TX_SIZE_16>   g_classic;
FlexCAN_T4FD<CAN3, RX_SIZE_64, TX_SIZE_16> g_fd;

// --- SPSC ring, RX ISR -> poll() -------------------------------------------
// trigger_engine.h pattern: power-of-two ring, ISR-side head, loop-side tail.
constexpr uint32_t kRxRingSize = 32;

struct RxFrame {
    uint64_t t_us = 0;  // arrival stamp (wrap-extended micros, ISR)
    uint32_t id   = 0;  // 29-bit arbitration id
    uint8_t  len  = 0;
    uint8_t  buf[canp::kFdChassisSpeedsLen] = {};
};

RxFrame           g_ring[kRxRingSize];
volatile uint32_t g_head = 0;
volatile uint32_t g_tail = 0;

volatile uint32_t g_isr_rx_drops = 0;

// Mirrors of the active mode for the ISRs (they have no object context).
void push_rx(uint32_t id, const uint8_t* buf, uint8_t len) {
    const uint32_t head = g_head;
    if (head - g_tail >= kRxRingSize) {
        ++g_isr_rx_drops;
        return;
    }
    RxFrame& f = g_ring[head & (kRxRingSize - 1)];
    f.t_us     = now_us64();
    f.id       = id;
    f.len      = (len > sizeof(f.buf)) ? sizeof(f.buf) : len;
    memcpy(f.buf, buf, f.len);
    g_head = head + 1;
}

void isr_rx_classic(const CAN_message_t& msg) {
    push_rx(msg.id, msg.buf, msg.len);
}

void isr_rx_fd(const CANFD_message_t& msg) {
    push_rx(msg.id, msg.buf, msg.len);
}

// --- main-loop state (poll() only — no locking needed) ---------------------
// Classic STAMP latch + RIO-clock wrap extension.
bool     g_have_stamp     = false;
uint8_t  g_stamp_counter  = 0;
uint64_t g_stamp_rio_us   = 0;
uint32_t g_rio_hi         = 0;
uint32_t g_rio_last_lo    = 0;

uint32_t g_rx_count       = 0;
uint32_t g_stamp_mismatch = 0;
uint32_t g_odom_tx_drops  = 0;
uint32_t g_pose_tx        = 0;
uint32_t g_pose_tx_drops  = 0;

void emit_odom(uint64_t t_arrival_us, const canp::ChassisSpeedsWire& s,
               uint8_t mode) {
    uint8_t payload[canp::kOdomTelemetryPayloadLen];
    canp::encode_odom_telemetry(t_arrival_us, s, mode, payload);

    uint8_t frame[6 + sizeof(payload)];
    const size_t n =
        build_frame(BinType::Odom, payload, sizeof(payload), frame);
    if (SerialUSB1.availableForWrite() >= static_cast<int>(n)) {
        SerialUSB1.write(frame, n);
    } else {
        ++g_odom_tx_drops;
    }
}

bool configure_common_classic() {
    g_classic.setMBFilter(REJECT_ALL);
    g_classic.setMB(MB0, RX, EXT);
    g_classic.setMB(MB1, RX, EXT);
    g_classic.setMB(MB8, TX, EXT);
    g_classic.setMB(MB9, TX, EXT);
    g_classic.setMBFilter(MB0, canp::kIdChassisSpeeds);
    g_classic.setMBFilter(MB1, canp::kIdChassisStamp);
    g_classic.onReceive(isr_rx_classic);
    g_classic.enableMBInterrupts();
    return true;
}

bool configure_common_fd(const char*& err) {
    // 4 Mbps data needs the 80 MHz FlexCAN clock — the default 24 MHz leaves
    // too few time quanta per bit for a sane sample point.
    CANFD_timings_t cfg;
    cfg.baudrate   = 1'000'000;
    cfg.baudrateFD = 4'000'000;
    cfg.clock      = CLK_80MHz;
    if (!g_fd.setBaudRate(cfg)) {
        err = "FD timing solution failed";
        return false;
    }
    g_fd.setRegions(32);  // 24-byte frames need >= 32-byte mailboxes
    g_fd.setMBFilter(REJECT_ALL);
    g_fd.setMB(MB0, RX, EXT);
    g_fd.setMB(MB8, TX, EXT);
    g_fd.setMBFilter(MB0, canp::kIdChassisSpeeds);
    g_fd.onReceive(isr_rx_fd);
    g_fd.enableMBInterrupts();
    return true;
}

}  // namespace

void CanBridge::begin() {
    // Nothing to do until the host pushes a mode; constructors already ran at
    // static init. Keeping a begin() hook preserves the subsystem pattern.
}

bool CanBridge::set_mode(CanMode mode, const char*& err) {
    if (mode == mode_ && configured_) return true;

    // Quiesce whichever controller is live. Both objects drive the same CAN3
    // peripheral, and each begin() soft-resets it from freeze mode and
    // reinstalls its own NVIC vector — so a live→live switch only needs the
    // interrupt mask cleared here. Runtime mode switching is still not an
    // officially supported library flow (bench-verified; power cycle is the
    // fallback). Going to Off leaves the controller configured but ignored
    // (FD's reset() is private); it keeps ACKing bus traffic until a power
    // cycle, which is harmless.
    if (mode_ == CanMode::Classic) {
        g_classic.disableMBInterrupts();
        if (mode == CanMode::Off) g_classic.reset();
    } else if (mode_ == CanMode::Fd) {
        g_fd.disableMBInterrupts();
    }
    mode_       = CanMode::Off;
    configured_ = false;
    g_have_stamp = false;

    if (mode == CanMode::Off) return true;

    // Both constructors set the per-TU ISR dispatch pointer `_CAN3 = this` at
    // static-init time, so whichever ran LAST owns the CAN3 vector regardless
    // of which object is begun. Point it at the object we're activating.
    if (mode == CanMode::Classic) {
        _CAN3 = &g_classic;
        g_classic.begin();
        g_classic.setBaudRate(1'000'000);
        configure_common_classic();
    } else {
        _CAN3 = &g_fd;
        g_fd.begin();
        if (!configure_common_fd(err)) {
            g_fd.disableMBInterrupts();
            return false;
        }
    }
    mode_       = mode;
    configured_ = true;
    return true;
}

void CanBridge::poll() {
    while (g_tail != g_head) {
        // Snapshot then advance: the ISR never touches g_tail, and slots
        // behind g_head are stable.
        const RxFrame f = g_ring[g_tail & (kRxRingSize - 1)];
        g_tail          = g_tail + 1;

        const uint8_t mode = static_cast<uint8_t>(mode_);

        if (f.id == canp::kIdChassisStamp) {
            uint32_t rio_lo  = 0;
            uint8_t  counter = 0;
            if (canp::decode_classic_stamp(f.buf, f.len, rio_lo, counter)) {
                g_stamp_rio_us =
                    canp::extend_u32(rio_lo, g_rio_hi, g_rio_last_lo);
                g_stamp_counter = counter;
                g_have_stamp    = true;
            }
            continue;
        }
        if (f.id != canp::kIdChassisSpeeds) continue;

        canp::ChassisSpeedsWire s;
        bool decoded = false;
        if (mode_ == CanMode::Fd) {
            decoded = canp::decode_fd_chassis_speeds(f.buf, f.len, s);
        } else {
            decoded = canp::decode_classic_chassis_speeds(f.buf, f.len, s);
            if (decoded) {
                if (g_have_stamp && g_stamp_counter == s.counter) {
                    s.rio_time_us = g_stamp_rio_us;
                } else {
                    // Lost or out-of-order STAMP — forward with the time
                    // unknown; the host falls back to the arrival stamp.
                    ++g_stamp_mismatch;
                }
                g_have_stamp = false;
            }
        }
        if (!decoded) continue;

        ++g_rx_count;
        emit_odom(f.t_us, s, mode);
    }
}

bool CanBridge::send_pose(const canp::PoseWire& pose) {
    if (mode_ == CanMode::Off || !configured_) return false;

    bool sent = false;
    if (mode_ == CanMode::Fd) {
        CANFD_message_t msg;
        msg.id             = canp::kIdPose;
        msg.flags.extended = 1;
        msg.brs            = 1;
        msg.edl            = 1;
        msg.len            = canp::kFdPoseLen;
        canp::encode_fd_pose(pose, msg.buf);
        sent = g_fd.write(msg) >= 1;
    } else {
        CAN_message_t xy;
        xy.id             = canp::kIdPoseXy;
        xy.flags.extended = 1;
        xy.len            = canp::kClassicFrameLen;
        canp::encode_classic_pose_xy(pose, xy.buf);

        CAN_message_t th;
        th.id             = canp::kIdPoseTheta;
        th.flags.extended = 1;
        th.len            = canp::kClassicFrameLen;
        canp::encode_classic_pose_theta(pose, th.buf);

        // XY first; the controller latches it and publishes on THETA.
        const bool a = g_classic.write(xy) >= 1;
        const bool b = g_classic.write(th) >= 1;
        sent         = a && b;
    }

    if (sent) ++g_pose_tx;
    else ++g_pose_tx_drops;
    return sent;
}

uint32_t CanBridge::rx_count() const { return g_rx_count; }
uint32_t CanBridge::rx_drops() const { return g_isr_rx_drops + g_stamp_mismatch; }
uint32_t CanBridge::odom_tx_drops() const { return g_odom_tx_drops; }
uint32_t CanBridge::pose_tx() const { return g_pose_tx; }
uint32_t CanBridge::pose_tx_drops() const { return g_pose_tx_drops; }

}  // namespace gw_fw
