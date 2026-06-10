#include "consumer/rosbag_writer.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace gw {

namespace {

// ROS1 op codes — see http://wiki.ros.org/Bags/Format/2.0#Records.
constexpr uint8_t kOpMessageData = 0x02;
constexpr uint8_t kOpBagHeader   = 0x03;
constexpr uint8_t kOpIndexData   = 0x04;
constexpr uint8_t kOpChunk       = 0x05;
constexpr uint8_t kOpChunkInfo   = 0x06;
constexpr uint8_t kOpConnection  = 0x07;

constexpr const char* kMagic = "#ROSBAG V2.0\n";

// Canonical MD5s — changing a message's fields would change these. They gate
// type compatibility in every ROS deserializer.
constexpr const char* kImageType = "sensor_msgs/Image";
constexpr const char* kImageMd5  = "060021388200f6f0f447d0fcd9c64743";
constexpr const char* kImuType   = "sensor_msgs/Imu";
constexpr const char* kImuMd5    = "6a62c6daae103f4ff57a132d6f95cec2";

uint32_t ns_to_sec(uint64_t ns) { return static_cast<uint32_t>(ns / 1'000'000'000ull); }
uint32_t ns_to_nsec(uint64_t ns) { return static_cast<uint32_t>(ns % 1'000'000'000ull); }

// ROS Time wire format: two consecutive uint32 LE values, sec then nsec.
void encode_time(uint8_t out[8], uint64_t ns) {
    const uint32_t s  = ns_to_sec(ns);
    const uint32_t ns_ = ns_to_nsec(ns);
    std::memcpy(out + 0, &s,   4);
    std::memcpy(out + 4, &ns_, 4);
}

// Canonical concatenated form: top-level message followed by every embedded
// type, '='*80 separators. rosbag's Python reader (which Kalibr uses) builds
// its deserializers from this text via genpy, so the embedded definitions
// are required, not documentation.
const std::string& image_message_definition() {
    static const std::string def =
        "std_msgs/Header header\n"
        "uint32 height\n"
        "uint32 width\n"
        "string encoding\n"
        "uint8 is_bigendian\n"
        "uint32 step\n"
        "uint8[] data\n"
        "\n"
        "================================================================================\n"
        "MSG: std_msgs/Header\n"
        "uint32 seq\n"
        "time stamp\n"
        "string frame_id\n";
    return def;
}

const std::string& imu_message_definition() {
    static const std::string def =
        "std_msgs/Header header\n"
        "geometry_msgs/Quaternion orientation\n"
        "float64[9] orientation_covariance\n"
        "geometry_msgs/Vector3 angular_velocity\n"
        "float64[9] angular_velocity_covariance\n"
        "geometry_msgs/Vector3 linear_acceleration\n"
        "float64[9] linear_acceleration_covariance\n"
        "\n"
        "================================================================================\n"
        "MSG: std_msgs/Header\n"
        "uint32 seq\n"
        "time stamp\n"
        "string frame_id\n"
        "\n"
        "================================================================================\n"
        "MSG: geometry_msgs/Quaternion\n"
        "float64 x\n"
        "float64 y\n"
        "float64 z\n"
        "float64 w\n"
        "\n"
        "================================================================================\n"
        "MSG: geometry_msgs/Vector3\n"
        "float64 x\n"
        "float64 y\n"
        "float64 z\n";
    return def;
}

}  // namespace

RosbagWriter::RosbagWriter(std::filesystem::path path,
                           std::string           topic_name,
                           std::string           frame_id)
    : path_(std::move(path)) {
    register_connection(ConnKind::Image, std::move(topic_name), std::move(frame_id));
}

RosbagWriter::~RosbagWriter() {
    if (opened_ && !closed_) {
        try { close(); } catch (...) {}
    }
}

uint32_t RosbagWriter::register_connection(ConnKind kind, std::string topic,
                                           std::string frame_id) {
    if (opened_) {
        throw std::runtime_error("RosbagWriter: connections must be registered before open()");
    }
    conns_.push_back(ConnectionInfo{kind, std::move(topic), std::move(frame_id), {}, 0});
    return static_cast<uint32_t>(conns_.size() - 1);
}

uint32_t RosbagWriter::add_image_connection(std::string topic_name, std::string frame_id) {
    return register_connection(ConnKind::Image, std::move(topic_name), std::move(frame_id));
}

uint32_t RosbagWriter::add_imu_connection(std::string topic_name, std::string frame_id) {
    return register_connection(ConnKind::Imu, std::move(topic_name), std::move(frame_id));
}

void RosbagWriter::put_u8(uint8_t v)   { out_.put(static_cast<char>(v)); }
void RosbagWriter::put_u32(uint32_t v) { out_.write(reinterpret_cast<const char*>(&v), 4); }
void RosbagWriter::put_u64(uint64_t v) { out_.write(reinterpret_cast<const char*>(&v), 8); }
void RosbagWriter::put_f64(double v)   { out_.write(reinterpret_cast<const char*>(&v), 8); }
void RosbagWriter::put_bytes(const void* data, size_t n) {
    out_.write(static_cast<const char*>(data), static_cast<std::streamsize>(n));
}

void RosbagWriter::put_field(std::string_view name,
                             const void*      value,
                             size_t           value_len) {
    const uint32_t field_len = static_cast<uint32_t>(name.size() + 1 + value_len);
    put_u32(field_len);
    put_bytes(name.data(), name.size());
    put_u8('=');
    put_bytes(value, value_len);
}

void RosbagWriter::put_field_u8(std::string_view name, uint8_t v) {
    put_field(name, &v, 1);
}
void RosbagWriter::put_field_u32(std::string_view name, uint32_t v) {
    put_field(name, &v, 4);
}
void RosbagWriter::put_field_u64(std::string_view name, uint64_t v) {
    put_field(name, &v, 8);
}
void RosbagWriter::put_field_str(std::string_view name, std::string_view value) {
    put_field(name, value.data(), value.size());
}

void RosbagWriter::write_magic() {
    out_.write(kMagic, static_cast<std::streamsize>(std::char_traits<char>::length(kMagic)));
}

void RosbagWriter::write_bag_header_placeholder() {
    // Fixed-size record so the placeholder can be overwritten in place on
    // close(): u32 hdr_len + header fields + u32 data_len + padding.
    // Total = kBagHeaderRecordBytes.
    bag_header_pos_ = static_cast<uint64_t>(out_.tellp());

    // Header field block: op, index_pos, conn_count, chunk_count.
    // We compute the exact header size so the data padding is well-defined.
    const uint32_t op_field_bytes          = 4 + (2 + 1 + 1);   // "op=\x03"
    const uint32_t index_pos_field_bytes   = 4 + (9 + 1 + 8);   // "index_pos=" + u64
    const uint32_t conn_count_field_bytes  = 4 + (10 + 1 + 4);  // "conn_count=" + u32
    const uint32_t chunk_count_field_bytes = 4 + (11 + 1 + 4);  // "chunk_count=" + u32
    const uint32_t hdr_len = op_field_bytes - 4 + index_pos_field_bytes - 4 +
                             conn_count_field_bytes - 4 + chunk_count_field_bytes - 4 +
                             // Re-add the 4-byte length prefix for each field:
                             4 * 4;

    const uint32_t fixed_overhead = 4 + hdr_len + 4;
    if (kBagHeaderRecordBytes < fixed_overhead) {
        throw std::runtime_error("RosbagWriter: BagHeader record budget too small");
    }
    const uint32_t data_padding = static_cast<uint32_t>(kBagHeaderRecordBytes - fixed_overhead);

    put_u32(hdr_len);
    put_field_u8 ("op",          kOpBagHeader);
    put_field_u64("index_pos",   0);            // placeholder, rewritten in close()
    put_field_u32("conn_count",  0);            // placeholder
    put_field_u32("chunk_count", 0);            // placeholder
    put_u32(data_padding);
    // Pad data block with spaces and terminate with newline (rosbag style).
    if (data_padding > 0) {
        std::string pad(data_padding, ' ');
        pad.back() = '\n';
        put_bytes(pad.data(), pad.size());
    }
}

void RosbagWriter::rewrite_bag_header(uint64_t index_pos,
                                      uint32_t conn_count,
                                      uint32_t chunk_count) {
    const auto save = out_.tellp();
    out_.seekp(static_cast<std::streamoff>(bag_header_pos_));

    // Recompute the same hdr_len we wrote in the placeholder.
    const uint32_t hdr_len = (2 + 1 + 1) + (9 + 1 + 8) +
                             (10 + 1 + 4) + (11 + 1 + 4) +
                             4 * 4;
    const uint32_t fixed_overhead = 4 + hdr_len + 4;
    const uint32_t data_padding = static_cast<uint32_t>(kBagHeaderRecordBytes - fixed_overhead);

    put_u32(hdr_len);
    put_field_u8 ("op",          kOpBagHeader);
    put_field_u64("index_pos",   index_pos);
    put_field_u32("conn_count",  conn_count);
    put_field_u32("chunk_count", chunk_count);
    put_u32(data_padding);
    // Padding bytes were already written; no need to re-emit them — the file
    // contents past this point in the placeholder are intact.

    out_.seekp(save);
}

void RosbagWriter::write_connection_record(uint32_t conn_id) {
    const ConnectionInfo& c = conns_[conn_id];

    // Connection record header: op, conn, topic.
    const uint32_t op_bytes    = 2 + 1 + 1;                   // op=\x07
    const uint32_t conn_bytes  = 4 + 1 + 4;                   // conn=<u32>
    const uint32_t topic_bytes = 5 + 1 + static_cast<uint32_t>(c.topic.size()); // topic=<str>
    const uint32_t hdr_len     = (4 + op_bytes) + (4 + conn_bytes) + (4 + topic_bytes);

    put_u32(hdr_len);
    put_field_u8 ("op",    kOpConnection);
    put_field_u32("conn",  conn_id);
    put_field_str("topic", c.topic);

    put_u32(static_cast<uint32_t>(c.data_block.size()));
    put_bytes(c.data_block.data(), c.data_block.size());
}

void RosbagWriter::write_message_header(uint32_t conn_id, uint64_t timestamp_ns) {
    // Record header: op, conn, time (8-byte ROS Time).
    const uint32_t op_bytes   = 2 + 1 + 1;       // op=\x02
    const uint32_t conn_bytes = 4 + 1 + 4;       // conn=<u32>
    const uint32_t time_bytes = 4 + 1 + 8;       // time=<8B>
    const uint32_t hdr_len    = (4 + op_bytes) + (4 + conn_bytes) + (4 + time_bytes);

    put_u32(hdr_len);
    put_field_u8 ("op",   kOpMessageData);
    put_field_u32("conn", conn_id);
    {
        uint8_t time_buf[8];
        encode_time(time_buf, timestamp_ns);
        put_field("time", time_buf, 8);
    }
}

void RosbagWriter::write_image_message(uint32_t       conn_id,
                                       uint64_t       timestamp_ns,
                                       uint32_t       width,
                                       uint32_t       height,
                                       const uint8_t* pixels) {
    ConnectionInfo& c = conns_[conn_id];
    write_message_header(conn_id, timestamp_ns);

    // Data block: serialized sensor_msgs/Image (little-endian).
    //   header.seq  : u32
    //   header.stamp: 8-byte Time
    //   header.frame_id: <u32 len><bytes>
    //   height      : u32
    //   width       : u32
    //   encoding    : "mono8" as <u32 len><bytes>
    //   is_bigendian: u8 = 0
    //   step        : u32 = width
    //   data        : <u32 len><width*height bytes>
    const uint32_t step       = width;
    const uint32_t pix_bytes  = step * height;
    const uint32_t frame_len  = static_cast<uint32_t>(c.frame_id.size());
    const char     enc[]      = "mono8";
    const uint32_t enc_len    = 5;
    const uint32_t data_len   =
        4 +                              // seq
        8 +                              // stamp
        4 + frame_len +                  // frame_id
        4 +                              // height
        4 +                              // width
        4 + enc_len +                    // encoding
        1 +                              // is_bigendian
        4 +                              // step
        4 + pix_bytes;                   // data
    put_u32(data_len);

    put_u32(c.seq++);
    {
        uint8_t time_buf[8];
        encode_time(time_buf, timestamp_ns);
        put_bytes(time_buf, 8);
    }
    put_u32(frame_len);
    put_bytes(c.frame_id.data(), frame_len);
    put_u32(height);
    put_u32(width);
    put_u32(enc_len);
    put_bytes(enc, enc_len);
    put_u8(0);                            // is_bigendian
    put_u32(step);
    put_u32(pix_bytes);
    put_bytes(pixels, pix_bytes);
}

void RosbagWriter::write_imu_message(uint32_t    conn_id,
                                     uint64_t    timestamp_ns,
                                     const float accel[3],
                                     const float gyro[3]) {
    ConnectionInfo& c = conns_[conn_id];
    write_message_header(conn_id, timestamp_ns);

    // Data block: serialized sensor_msgs/Imu (little-endian, all f64).
    //   header (seq u32, stamp 8B, frame_id <len><bytes>)
    //   orientation: quaternion x,y,z,w — identity (we publish none)
    //   orientation_covariance[9] — [0] = -1 marks "no orientation estimate"
    //   angular_velocity x,y,z (rad/s) + covariance[9] = zeros (unknown)
    //   linear_acceleration x,y,z (m/s²) + covariance[9] = zeros (unknown)
    const uint32_t frame_len = static_cast<uint32_t>(c.frame_id.size());
    const uint32_t data_len  =
        4 + 8 + 4 + frame_len +          // header
        4 * 8 +                          // orientation quaternion
        9 * 8 +                          // orientation_covariance
        3 * 8 + 9 * 8 +                  // angular_velocity + covariance
        3 * 8 + 9 * 8;                   // linear_acceleration + covariance
    put_u32(data_len);

    put_u32(c.seq++);
    {
        uint8_t time_buf[8];
        encode_time(time_buf, timestamp_ns);
        put_bytes(time_buf, 8);
    }
    put_u32(frame_len);
    put_bytes(c.frame_id.data(), frame_len);

    put_f64(0.0); put_f64(0.0); put_f64(0.0); put_f64(1.0);   // orientation
    put_f64(-1.0);                                            // orientation_cov[0]
    for (int i = 1; i < 9; ++i) put_f64(0.0);
    for (int i = 0; i < 3; ++i) put_f64(static_cast<double>(gyro[i]));
    for (int i = 0; i < 9; ++i) put_f64(0.0);
    for (int i = 0; i < 3; ++i) put_f64(static_cast<double>(accel[i]));
    for (int i = 0; i < 9; ++i) put_f64(0.0);
}

void RosbagWriter::begin_chunk() {
    chunks_.emplace_back();
    cur_chunk_ = &chunks_.back();
    cur_chunk_->file_pos = static_cast<uint64_t>(out_.tellp());
    cur_chunk_->index_per_conn.resize(conns_.size());

    // Chunk record header: op, compression, size (placeholder).
    const uint32_t op_bytes    = 2 + 1 + 1;                                       // op=\x05
    const std::string comp     = "none";
    const uint32_t comp_bytes  = 11 + 1 + static_cast<uint32_t>(comp.size());     // compression=none
    const uint32_t size_bytes  = 4 + 1 + 4;                                       // size=<u32>
    const uint32_t hdr_len     = (4 + op_bytes) + (4 + comp_bytes) + (4 + size_bytes);

    put_u32(hdr_len);
    put_field_u8 ("op",          kOpChunk);
    put_field_str("compression", comp);
    put_field_u32("size",        0);    // placeholder, patched in end_chunk()

    put_u32(0);                          // data_len placeholder
    cur_chunk_->data_start_pos = static_cast<uint64_t>(out_.tellp());

    // Connection records at the top of the chunk's data block — every chunk
    // is self-describing, which matches rosbag's standard layout.
    for (uint32_t id = 0; id < conns_.size(); ++id) {
        write_connection_record(id);
    }
}

void RosbagWriter::end_chunk() {
    if (!cur_chunk_) return;
    const uint64_t data_end_pos = static_cast<uint64_t>(out_.tellp());
    const uint64_t data_bytes_64 = data_end_pos - cur_chunk_->data_start_pos;
    if (data_bytes_64 > UINT32_MAX) {
        throw std::runtime_error("RosbagWriter: chunk exceeded 4 GB");
    }
    const uint32_t data_bytes = static_cast<uint32_t>(data_bytes_64);

    // Patch in `size=<data_bytes>` and the record's data_len.
    //
    // Chunk record layout written by begin_chunk():
    //   [u32 hdr_len][op field][compression field][size field][u32 data_len]
    // We seek to the size field's value bytes and to the data_len u32.
    const uint64_t hdr_len_pos = cur_chunk_->file_pos;             // u32 hdr_len
    const uint64_t op_field_start = hdr_len_pos + 4;               // first field record
    // op field: u32 len + "op=\x03"
    const uint64_t comp_field_start = op_field_start + 4 + (2 + 1 + 1);
    // compression field: u32 len + "compression=none"
    const uint64_t size_field_start = comp_field_start + 4 + (11 + 1 + 4);
    // size field: u32 len + "size=<u32 value>"
    //   prefix u32 + "size=" + 4 bytes value
    const uint64_t size_value_pos = size_field_start + 4 + 4 + 1;  // u32 len + "size=" (5 bytes)
    // data_len comes right after the header block:
    const uint64_t data_len_pos = cur_chunk_->data_start_pos - 4;

    const auto save = out_.tellp();
    out_.seekp(static_cast<std::streamoff>(size_value_pos));
    put_u32(data_bytes);
    out_.seekp(static_cast<std::streamoff>(data_len_pos));
    put_u32(data_bytes);
    out_.seekp(save);

    // One IndexData record per registered connection (written even when the
    // connection logged no messages this chunk — matches rosbag and keeps
    // the N=1 output identical to the original single-topic writer).
    for (uint32_t id = 0; id < conns_.size(); ++id) {
        auto& index = cur_chunk_->index_per_conn[id];
        std::stable_sort(index.begin(), index.end(),
                         [](const IndexEntry& a, const IndexEntry& b) {
                             return a.timestamp_ns < b.timestamp_ns;
                         });
        const uint32_t count = static_cast<uint32_t>(index.size());

        const uint32_t op_bytes_    = 2 + 1 + 1;            // op=\x04
        const uint32_t ver_bytes    = 3 + 1 + 4;            // ver=<u32>
        const uint32_t conn_bytes   = 4 + 1 + 4;            // conn=<u32>
        const uint32_t count_bytes  = 5 + 1 + 4;            // count=<u32>
        const uint32_t hdr_len      = (4 + op_bytes_) + (4 + ver_bytes) +
                                      (4 + conn_bytes) + (4 + count_bytes);
        put_u32(hdr_len);
        put_field_u8 ("op",    kOpIndexData);
        put_field_u32("ver",   1);
        put_field_u32("conn",  id);
        put_field_u32("count", count);

        // Entry: 8-byte time + 4-byte offset within chunk data block.
        const uint32_t data_len = 12u * count;
        put_u32(data_len);
        for (const auto& e : index) {
            uint8_t time_buf[8];
            encode_time(time_buf, e.timestamp_ns);
            put_bytes(time_buf, 8);
            put_u32(e.chunk_offset);
        }
    }
    cur_chunk_ = nullptr;
}

void RosbagWriter::open() {
    if (opened_) return;

    std::error_code ec;
    std::filesystem::create_directories(path_.parent_path(), ec);
    // ignore ec: ofstream open will fail if parent really doesn't exist

    out_.open(path_, std::ios::out | std::ios::binary | std::ios::trunc);
    if (!out_.is_open()) {
        throw std::runtime_error("RosbagWriter: cannot open " + path_.string());
    }
    opened_ = true;
    closed_ = false;
    msg_count_ = 0;
    chunks_.clear();
    cur_chunk_ = nullptr;

    // Pre-serialize each Connection record's data block (itself a field
    // block: <u32 len><k=v> × 4). Topic, type, md5, and message_definition
    // are immutable after open(), so the same byte sequence is re-emitted at
    // the top of every chunk and once in the trailing index region.
    auto append_field = [](std::string& out, std::string_view k, std::string_view v) {
        const uint32_t len = static_cast<uint32_t>(k.size() + 1 + v.size());
        out.append(reinterpret_cast<const char*>(&len), 4);
        out.append(k);
        out.push_back('=');
        out.append(v);
    };
    for (auto& c : conns_) {
        const bool  imu  = (c.kind == ConnKind::Imu);
        const char* type = imu ? kImuType : kImageType;
        const char* md5  = imu ? kImuMd5 : kImageMd5;
        const std::string& def =
            imu ? imu_message_definition() : image_message_definition();
        c.data_block.clear();
        c.seq = 0;
        append_field(c.data_block, "topic",              c.topic);
        append_field(c.data_block, "type",               type);
        append_field(c.data_block, "md5sum",             md5);
        append_field(c.data_block, "message_definition", def);
    }

    write_magic();
    write_bag_header_placeholder();
    begin_chunk();
}

uint32_t RosbagWriter::prepare_message(uint32_t conn_id, ConnKind expected_kind,
                                       uint64_t approx_record_bytes) {
    if (!opened_ || closed_) {
        throw std::runtime_error("RosbagWriter: not open");
    }
    if (conn_id >= conns_.size()) {
        throw std::runtime_error("RosbagWriter: unknown conn_id");
    }
    if (conns_[conn_id].kind != expected_kind) {
        throw std::runtime_error("RosbagWriter: message type does not match connection");
    }
    if (!cur_chunk_) begin_chunk();

    uint64_t cur_pos = static_cast<uint64_t>(out_.tellp());
    if (cur_pos - cur_chunk_->data_start_pos + approx_record_bytes > kChunkBudget) {
        end_chunk();
        begin_chunk();
        cur_pos = static_cast<uint64_t>(out_.tellp());
    }
    return static_cast<uint32_t>(cur_pos - cur_chunk_->data_start_pos);
}

void RosbagWriter::note_message(uint32_t conn_id, uint64_t timestamp_ns,
                                uint32_t chunk_offset) {
    cur_chunk_->index_per_conn[conn_id].push_back({timestamp_ns, chunk_offset});
    cur_chunk_->start_time_ns = std::min(cur_chunk_->start_time_ns, timestamp_ns);
    cur_chunk_->end_time_ns   = std::max(cur_chunk_->end_time_ns,   timestamp_ns);
    msg_count_++;
}

void RosbagWriter::add_mono8_image(uint32_t       conn_id,
                                   uint64_t       timestamp_ns,
                                   uint32_t       width,
                                   uint32_t       height,
                                   const uint8_t* pixels) {
    const uint64_t pix_bytes = static_cast<uint64_t>(width) * static_cast<uint64_t>(height);
    const uint32_t offset = prepare_message(conn_id, ConnKind::Image, 64 + pix_bytes);
    write_image_message(conn_id, timestamp_ns, width, height, pixels);
    note_message(conn_id, timestamp_ns, offset);
}

void RosbagWriter::add_mono8_image(uint64_t       timestamp_ns,
                                   uint32_t       width,
                                   uint32_t       height,
                                   const uint8_t* pixels) {
    add_mono8_image(0, timestamp_ns, width, height, pixels);
}

void RosbagWriter::add_imu_sample(uint32_t    conn_id,
                                  uint64_t    timestamp_ns,
                                  const float accel[3],
                                  const float gyro[3]) {
    const uint32_t offset = prepare_message(conn_id, ConnKind::Imu, 64 + 33 * 8);
    write_imu_message(conn_id, timestamp_ns, accel, gyro);
    note_message(conn_id, timestamp_ns, offset);
}

void RosbagWriter::close() {
    if (!opened_ || closed_) return;

    end_chunk();

    // Trailing index region: Connection records first, then ChunkInfo records.
    // BagHeader.index_pos must point at the *start* of this region (the first
    // Connection record). rosbag's reader seeks here, calls read_connection_
    // record() expecting op=CONNECTION, and aborts with "Unindexed bag" if it
    // hits a ChunkInfo instead — so capture the offset BEFORE the Connection
    // writes, not after.
    const uint64_t index_pos = static_cast<uint64_t>(out_.tellp());
    for (uint32_t id = 0; id < conns_.size(); ++id) {
        write_connection_record(id);
    }

    // ChunkInfo records — one per chunk we wrote.
    for (const auto& c : chunks_) {
        const uint32_t op_bytes        = 2 + 1 + 1;             // op=\x06
        const uint32_t ver_bytes       = 3 + 1 + 4;             // ver=<u32>
        const uint32_t chunk_pos_bytes = 9 + 1 + 8;             // chunk_pos=<u64>
        const uint32_t start_bytes     = 10 + 1 + 8;            // start_time=<8B>
        const uint32_t end_bytes       = 8 + 1 + 8;             // end_time=<8B>
        const uint32_t count_bytes     = 5 + 1 + 4;             // count=<u32>
        const uint32_t hdr_len         =
            (4 + op_bytes) + (4 + ver_bytes) + (4 + chunk_pos_bytes) +
            (4 + start_bytes) + (4 + end_bytes) + (4 + count_bytes);

        put_u32(hdr_len);
        put_field_u8 ("op",         kOpChunkInfo);
        put_field_u32("ver",        1);
        put_field_u64("chunk_pos",  c.file_pos);
        {
            uint8_t buf[8];
            encode_time(buf, c.start_time_ns == UINT64_MAX ? 0 : c.start_time_ns);
            put_field("start_time", buf, 8);
            encode_time(buf, c.end_time_ns);
            put_field("end_time",   buf, 8);
        }
        put_field_u32("count", static_cast<uint32_t>(conns_.size()));

        // Data block: <conn u32><msg_count u32> for each registered connection.
        put_u32(static_cast<uint32_t>(8 * conns_.size()));
        for (uint32_t id = 0; id < conns_.size(); ++id) {
            put_u32(id);
            put_u32(static_cast<uint32_t>(c.index_per_conn[id].size()));
        }
    }

    // Now go back and write the real BagHeader.
    rewrite_bag_header(index_pos, static_cast<uint32_t>(conns_.size()),
                       /*chunk_count=*/static_cast<uint32_t>(chunks_.size()));

    out_.flush();
    out_.close();
    closed_ = true;
}

}  // namespace gw
