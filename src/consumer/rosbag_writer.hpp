#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace gw {

// Minimal ROS1 v2.0 bag writer specialized for a single sensor_msgs/Image
// topic in Mono8 encoding. We write this format because Kalibr's
// kalibr_calibrate_cameras consumes ROS1 bags; producing them in-app saves us
// from depending on a full ROS install on macOS.
//
// Format reference: http://wiki.ros.org/Bags/Format/2.0
//
// What we deliberately don't support:
//   - Multiple topics / connections per bag.
//   - Compression (PoC keeps the writer simple; a 30 s mono recording fits in
//     ~700 MB uncompressed and Kalibr is happy with it).
//   - Reading the bag back — Kalibr is the only consumer.
//
// Not thread-safe: the writer holds an ofstream and a single-chunk buffer;
// the calling code (RosbagRecordingConsumer) uses one writer thread.
class RosbagWriter {
public:
    explicit RosbagWriter(std::filesystem::path path,
                          std::string           topic_name = "/cam0/image_raw",
                          std::string           frame_id   = "cam0");
    ~RosbagWriter();

    RosbagWriter(const RosbagWriter&)            = delete;
    RosbagWriter& operator=(const RosbagWriter&) = delete;

    // Opens the file, writes the magic + a padded placeholder BagHeader, and
    // begins the first chunk. Throws std::runtime_error on I/O failure.
    void open();

    // Appends one Mono8 image message. timestamp_ns must increase across
    // calls (Kalibr uses header.stamp to order frames). pixels points to a
    // tightly-packed width*height byte buffer.
    void add_mono8_image(uint64_t       timestamp_ns,
                         uint32_t       width,
                         uint32_t       height,
                         const uint8_t* pixels);

    // Closes the current chunk, writes the trailing connection + chunk-info
    // index, rewrites the BagHeader with the real index_pos, and closes the
    // file. Idempotent; the destructor calls it if the caller didn't.
    void close();

    uint64_t messages_written() const { return msg_count_; }

private:
    struct IndexEntry {
        uint64_t timestamp_ns;  // packed sec*1e9 + nsec
        uint32_t chunk_offset;  // bytes from start of chunk's data block
    };

    struct ChunkRecord {
        uint64_t                file_pos;       // record start in file
        uint64_t                data_start_pos; // first byte of data block
        uint64_t                start_time_ns = UINT64_MAX;
        uint64_t                end_time_ns   = 0;
        uint32_t                msg_count     = 0;
        std::vector<IndexEntry> index;
    };

    // Roll a new chunk when adding the next message would push the current
    // chunk over this budget. uint32 size field caps at ~4 GB; this keeps
    // ChunkInfo metadata small and avoids huge contiguous fwrite chunks.
    static constexpr uint64_t kChunkBudget = 256 * 1024 * 1024;  // 256 MB

    // Number of bytes reserved between the magic and the first chunk for the
    // padded BagHeader record. The header fields are fixed size, so we know
    // the exact record size up-front; we pad the data block with spaces so
    // the offset of the first chunk doesn't shift between open() and close().
    static constexpr uint64_t kBagHeaderRecordBytes = 4096;

    // Topic-/type-related constants. sensor_msgs/Image MD5 is canonical;
    // changing the encoding or removing fields would change it. The
    // message_definition is the standard concatenated form ROS emits.
    static constexpr const char* kTopicType = "sensor_msgs/Image";
    static constexpr const char* kTopicMd5  = "060021388200f6f0f447d0fcd9c64743";
    static const std::string&    image_message_definition();

    // Low-level helpers — all little-endian.
    void put_u8 (uint8_t v);
    void put_u32(uint32_t v);
    void put_u64(uint64_t v);
    void put_bytes(const void* data, size_t n);

    // Writes a single field record: <u32 field_len> "<name>=<value bytes>".
    // value_bytes may contain arbitrary binary; name must not contain '='.
    void put_field(std::string_view name, const void* value, size_t value_len);
    void put_field_u8 (std::string_view name, uint8_t  v);
    void put_field_u32(std::string_view name, uint32_t v);
    void put_field_u64(std::string_view name, uint64_t v);
    void put_field_str(std::string_view name, std::string_view value);

    void write_magic();
    void write_bag_header_placeholder();
    void rewrite_bag_header(uint64_t index_pos, uint32_t conn_count, uint32_t chunk_count);

    // Begin a new chunk: placeholder chunk header, then a Connection record
    // inside the chunk's data block (each chunk is self-describing).
    void begin_chunk();
    // Patch the chunk's size and data_len fields with the actual byte count,
    // then write the per-chunk IndexData record at the current file position.
    void end_chunk();

    void write_connection_record(uint32_t conn_id);
    void write_image_message(uint64_t timestamp_ns,
                             uint32_t width, uint32_t height,
                             const uint8_t* pixels);

    std::filesystem::path path_;
    std::string           topic_;
    std::string           frame_id_;
    std::ofstream         out_;
    bool                  opened_ = false;
    bool                  closed_ = false;

    uint64_t                  msg_count_ = 0;
    std::vector<ChunkRecord>  chunks_;
    ChunkRecord*              cur_chunk_ = nullptr;
    uint64_t                  bag_header_pos_ = 0;
    std::string               conn_data_block_;  // pre-serialized Connection record data
};

}  // namespace gw
