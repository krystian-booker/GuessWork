#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace gw {

// Minimal ROS1 v2.0 bag writer for Mono8 sensor_msgs/Image topics and
// sensor_msgs/Imu topics. We write this format because Kalibr consumes ROS1
// bags; producing them in-app saves us from depending on a full ROS install
// on macOS.
//
// Format reference: http://wiki.ros.org/Bags/Format/2.0
//
// The constructor registers the first image connection (conn 0); further
// topics are registered up-front via add_image_connection /
// add_imu_connection (before open()) and addressed by the returned conn_id.
// The defaulted constructor arguments and the conn-less add_mono8_image
// overload keep the original intrinsics-recording call sites working and
// produce byte-identical output for the one-image-topic case.
//
// What we deliberately don't support:
//   - Compression (a 30 s mono recording fits in ~700 MB uncompressed and
//     Kalibr is happy with it).
//   - Reading the bag back — Kalibr is the only consumer.
//
// Per-connection timestamps must be non-decreasing (rosbag readers merge
// per-connection indexes by time; cross-connection order is free). The
// per-chunk index entries are sorted defensively before writing.
//
// Not thread-safe: the writer holds an ofstream and per-chunk state; the
// calling code uses one writer thread.
class RosbagWriter {
public:
    // Registers one image connection (conn 0) for topic_name/frame_id.
    explicit RosbagWriter(std::filesystem::path path,
                          std::string           topic_name = "/cam0/image_raw",
                          std::string           frame_id   = "cam0");

    ~RosbagWriter();

    RosbagWriter(const RosbagWriter&)            = delete;
    RosbagWriter& operator=(const RosbagWriter&) = delete;

    // Register a topic. Must be called before open(); throws afterwards.
    // Returns the conn_id to pass to the add_* methods. Connection ids are
    // dense and assigned in registration order (0, 1, 2, …).
    uint32_t add_image_connection(std::string topic_name, std::string frame_id);
    uint32_t add_imu_connection(std::string topic_name, std::string frame_id);

    // Opens the file, writes the magic + a padded placeholder BagHeader, and
    // begins the first chunk. Requires at least one registered connection.
    // Throws std::runtime_error on I/O failure.
    void open();

    // Appends one Mono8 image message on an image connection. pixels points
    // to a tightly-packed width*height byte buffer.
    void add_mono8_image(uint32_t       conn_id,
                         uint64_t       timestamp_ns,
                         uint32_t       width,
                         uint32_t       height,
                         const uint8_t* pixels);

    // Legacy form: conn 0 (must be an image connection).
    void add_mono8_image(uint64_t       timestamp_ns,
                         uint32_t       width,
                         uint32_t       height,
                         const uint8_t* pixels);

    // Appends one sensor_msgs/Imu message on an IMU connection. accel is
    // m/s², gyro is rad/s. Orientation is left unset per the ROS convention
    // (identity quaternion, orientation_covariance[0] = -1).
    void add_imu_sample(uint32_t    conn_id,
                        uint64_t    timestamp_ns,
                        const float accel[3],
                        const float gyro[3]);

    // Closes the current chunk, writes the trailing connection + chunk-info
    // index, rewrites the BagHeader with the real index_pos, and closes the
    // file. Idempotent; the destructor calls it if the caller didn't.
    void close();

    uint64_t messages_written() const { return msg_count_; }

private:
    enum class ConnKind : uint8_t { Image, Imu };

    struct ConnectionInfo {
        ConnKind    kind;
        std::string topic;
        std::string frame_id;
        std::string data_block;  // pre-serialized Connection record data (open())
        uint32_t    seq = 0;     // per-connection header.seq counter
    };

    struct IndexEntry {
        uint64_t timestamp_ns;  // packed sec*1e9 + nsec
        uint32_t chunk_offset;  // bytes from start of chunk's data block
    };

    struct ChunkRecord {
        uint64_t file_pos;       // record start in file
        uint64_t data_start_pos; // first byte of data block
        uint64_t start_time_ns = UINT64_MAX;
        uint64_t end_time_ns   = 0;
        // One index per registered connection (same indices as conns_).
        std::vector<std::vector<IndexEntry>> index_per_conn;
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

    uint32_t register_connection(ConnKind kind, std::string topic, std::string frame_id);

    // Low-level helpers — all little-endian.
    void put_u8 (uint8_t v);
    void put_u32(uint32_t v);
    void put_u64(uint64_t v);
    void put_f64(double v);
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

    // Begin a new chunk: placeholder chunk header, then one Connection record
    // per registered connection inside the chunk's data block (each chunk is
    // self-describing).
    void begin_chunk();
    // Patch the chunk's size and data_len fields with the actual byte count,
    // then write one per-connection IndexData record at the current position.
    void end_chunk();

    void write_connection_record(uint32_t conn_id);
    void write_message_header(uint32_t conn_id, uint64_t timestamp_ns);
    void write_image_message(uint32_t conn_id, uint64_t timestamp_ns,
                             uint32_t width, uint32_t height,
                             const uint8_t* pixels);
    void write_imu_message(uint32_t conn_id, uint64_t timestamp_ns,
                           const float accel[3], const float gyro[3]);

    // Shared add_* preamble: validates state/conn, rolls the chunk if the
    // next record would exceed the budget, returns the record's offset
    // within the current chunk's data block.
    uint32_t prepare_message(uint32_t conn_id, ConnKind expected_kind,
                             uint64_t approx_record_bytes);
    void     note_message(uint32_t conn_id, uint64_t timestamp_ns, uint32_t chunk_offset);

    std::filesystem::path       path_;
    std::vector<ConnectionInfo> conns_;
    std::ofstream               out_;
    bool                        opened_ = false;
    bool                        closed_ = false;

    uint64_t                  msg_count_ = 0;
    std::vector<ChunkRecord>  chunks_;
    ChunkRecord*              cur_chunk_ = nullptr;
    uint64_t                  bag_header_pos_ = 0;
};

}  // namespace gw
