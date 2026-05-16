#include "consumer/rosbag_writer.hpp"

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

// One ROS connection per bag (we only emit /cam0/image_raw).
constexpr uint32_t kConnId = 0;

uint32_t ns_to_sec(uint64_t ns) { return static_cast<uint32_t>(ns / 1'000'000'000ull); }
uint32_t ns_to_nsec(uint64_t ns) { return static_cast<uint32_t>(ns % 1'000'000'000ull); }

// ROS Time wire format: two consecutive uint32 LE values, sec then nsec.
void encode_time(uint8_t out[8], uint64_t ns) {
    const uint32_t s  = ns_to_sec(ns);
    const uint32_t ns_ = ns_to_nsec(ns);
    std::memcpy(out + 0, &s,   4);
    std::memcpy(out + 4, &ns_, 4);
}

}  // namespace

const std::string& RosbagWriter::image_message_definition() {
    // Canonical concatenated form: sensor_msgs/Image + embedded
    // std_msgs/Header. Comments stripped — kalibr / rosbag only use this
    // string for description; the MD5 is what gates type compatibility.
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

RosbagWriter::RosbagWriter(std::filesystem::path path,
                           std::string           topic_name,
                           std::string           frame_id)
    : path_(std::move(path)),
      topic_(std::move(topic_name)),
      frame_id_(std::move(frame_id)) {}

RosbagWriter::~RosbagWriter() {
    if (opened_ && !closed_) {
        try { close(); } catch (...) {}
    }
}

void RosbagWriter::put_u8(uint8_t v)   { out_.put(static_cast<char>(v)); }
void RosbagWriter::put_u32(uint32_t v) { out_.write(reinterpret_cast<const char*>(&v), 4); }
void RosbagWriter::put_u64(uint64_t v) { out_.write(reinterpret_cast<const char*>(&v), 8); }
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
    // Sanity: hdr_len should be the sum of bytes between hdr_len prefix and
    // data_len prefix.

    // Total record bytes consumed so far if we use this hdr_len:
    //   4 (hdr_len prefix) + hdr_len + 4 (data_len prefix) + data_padding
    // We want the total to equal kBagHeaderRecordBytes.
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
    // Connection record header: op, conn, topic.
    const uint32_t op_bytes    = 2 + 1 + 1;                   // op=\x07
    const uint32_t conn_bytes  = 4 + 1 + 4;                   // conn=<u32>
    const uint32_t topic_bytes = 5 + 1 + static_cast<uint32_t>(topic_.size()); // topic=<str>
    const uint32_t hdr_len     = (4 + op_bytes) + (4 + conn_bytes) + (4 + topic_bytes);

    put_u32(hdr_len);
    put_field_u8 ("op",    kOpConnection);
    put_field_u32("conn",  conn_id);
    put_field_str("topic", topic_);

    put_u32(static_cast<uint32_t>(conn_data_block_.size()));
    put_bytes(conn_data_block_.data(), conn_data_block_.size());
}

void RosbagWriter::write_image_message(uint64_t       timestamp_ns,
                                       uint32_t       width,
                                       uint32_t       height,
                                       const uint8_t* pixels) {
    // Record header: op, conn, time (8-byte ROS Time).
    const uint32_t op_bytes   = 2 + 1 + 1;       // op=\x02
    const uint32_t conn_bytes = 4 + 1 + 4;       // conn=<u32>
    const uint32_t time_bytes = 4 + 1 + 8;       // time=<8B>
    const uint32_t hdr_len    = (4 + op_bytes) + (4 + conn_bytes) + (4 + time_bytes);

    put_u32(hdr_len);
    put_field_u8 ("op",   kOpMessageData);
    put_field_u32("conn", kConnId);
    {
        uint8_t time_buf[8];
        encode_time(time_buf, timestamp_ns);
        put_field("time", time_buf, 8);
    }

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
    const uint32_t frame_len  = static_cast<uint32_t>(frame_id_.size());
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

    const uint32_t seq = static_cast<uint32_t>(msg_count_ & 0xFFFFFFFFu);
    put_u32(seq);
    {
        uint8_t time_buf[8];
        encode_time(time_buf, timestamp_ns);
        put_bytes(time_buf, 8);
    }
    put_u32(frame_len);
    put_bytes(frame_id_.data(), frame_len);
    put_u32(height);
    put_u32(width);
    put_u32(enc_len);
    put_bytes(enc, enc_len);
    put_u8(0);                            // is_bigendian
    put_u32(step);
    put_u32(pix_bytes);
    put_bytes(pixels, pix_bytes);
}

void RosbagWriter::begin_chunk() {
    chunks_.emplace_back();
    cur_chunk_ = &chunks_.back();
    cur_chunk_->file_pos = static_cast<uint64_t>(out_.tellp());

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

    // Connection record at the top of the chunk's data block — every chunk
    // is self-describing, which matches rosbag's standard layout.
    write_connection_record(kConnId);
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

    // Write the IndexData record for this chunk: one record per connection.
    const uint32_t count = cur_chunk_->msg_count;
    {
        const uint32_t op_bytes_    = 2 + 1 + 1;            // op=\x04
        const uint32_t ver_bytes    = 3 + 1 + 4;            // ver=<u32>
        const uint32_t conn_bytes   = 4 + 1 + 4;            // conn=<u32>
        const uint32_t count_bytes  = 5 + 1 + 4;            // count=<u32>
        const uint32_t hdr_len      = (4 + op_bytes_) + (4 + ver_bytes) +
                                      (4 + conn_bytes) + (4 + count_bytes);
        put_u32(hdr_len);
        put_field_u8 ("op",    kOpIndexData);
        put_field_u32("ver",   1);
        put_field_u32("conn",  kConnId);
        put_field_u32("count", count);

        // Entry: 8-byte time + 4-byte offset within chunk data block.
        const uint32_t data_len = 12u * count;
        put_u32(data_len);
        for (const auto& e : cur_chunk_->index) {
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

    // Pre-serialize the Connection record's data block (itself a field block:
    // <u32 len><k=v> × 4). Topic, type, md5, and message_definition are
    // immutable after construction, so this same byte sequence is re-emitted
    // at the top of every chunk and once in the trailing index region.
    conn_data_block_.clear();
    auto append_field = [](std::string& out, std::string_view k, std::string_view v) {
        const uint32_t len = static_cast<uint32_t>(k.size() + 1 + v.size());
        out.append(reinterpret_cast<const char*>(&len), 4);
        out.append(k);
        out.push_back('=');
        out.append(v);
    };
    append_field(conn_data_block_, "topic",              topic_);
    append_field(conn_data_block_, "type",               kTopicType);
    append_field(conn_data_block_, "md5sum",             kTopicMd5);
    append_field(conn_data_block_, "message_definition", image_message_definition());

    write_magic();
    write_bag_header_placeholder();
    begin_chunk();
}

void RosbagWriter::add_mono8_image(uint64_t       timestamp_ns,
                                   uint32_t       width,
                                   uint32_t       height,
                                   const uint8_t* pixels) {
    if (!opened_ || closed_) {
        throw std::runtime_error("RosbagWriter: not open");
    }
    if (!cur_chunk_) begin_chunk();

    // Estimate this message's record size to decide whether to roll a chunk.
    const uint64_t pix_bytes = static_cast<uint64_t>(width) * static_cast<uint64_t>(height);
    const uint64_t approx_msg_record = 64 + pix_bytes;  // ~header + small fixed fields
    uint64_t cur_pos = static_cast<uint64_t>(out_.tellp());
    if (cur_pos - cur_chunk_->data_start_pos + approx_msg_record > kChunkBudget) {
        end_chunk();
        begin_chunk();
        cur_pos = static_cast<uint64_t>(out_.tellp());
    }

    const uint32_t offset_in_chunk =
        static_cast<uint32_t>(cur_pos - cur_chunk_->data_start_pos);
    write_image_message(timestamp_ns, width, height, pixels);

    cur_chunk_->index.push_back({timestamp_ns, offset_in_chunk});
    cur_chunk_->msg_count++;
    cur_chunk_->start_time_ns = std::min(cur_chunk_->start_time_ns, timestamp_ns);
    cur_chunk_->end_time_ns   = std::max(cur_chunk_->end_time_ns,   timestamp_ns);
    msg_count_++;
}

void RosbagWriter::close() {
    if (!opened_ || closed_) return;

    end_chunk();

    // Trailing index region: Connection records first, then ChunkInfo records.
    // BagHeader.index_pos must point at the *start* of this region (the first
    // Connection record). rosbag's reader seeks here, calls read_connection_
    // record() expecting op=CONNECTION, and aborts with "Unindexed bag" if it
    // hits a ChunkInfo instead — so capture the offset BEFORE the Connection
    // write, not after.
    const uint64_t index_pos = static_cast<uint64_t>(out_.tellp());
    write_connection_record(kConnId);

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
        put_field_u32("count", 1);                    // number of connections in this chunk

        // Data block: <conn u32><msg_count u32> for each connection (just one).
        put_u32(8);
        put_u32(kConnId);
        put_u32(c.msg_count);
    }

    // Now go back and write the real BagHeader.
    rewrite_bag_header(index_pos, /*conn_count=*/1,
                       /*chunk_count=*/static_cast<uint32_t>(chunks_.size()));

    out_.flush();
    out_.close();
    closed_ = true;
}

}  // namespace gw
