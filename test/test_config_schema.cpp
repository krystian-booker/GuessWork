#include <chrono>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <sqlite3.h>

#include "posest/config/ConfigValidator.h"
#include "posest/config/InMemoryConfigStore.h"
#include "posest/config/SqliteConfigStore.h"
#include "posest/config/SqliteSchema.h"
#include "posest/teensy/FakeSerialTransport.h"
#include "posest/teensy/Protocol.h"
#include "posest/teensy/TeensyService.h"

namespace {

std::filesystem::path tempDbPath(const std::string& name) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() /
           ("posest_" + name + "_" + std::to_string(stamp) + ".db");
}

int readUserVersion(const std::filesystem::path& path) {
    sqlite3* db = nullptr;
    if (sqlite3_open(path.string().c_str(), &db) != SQLITE_OK) {
        if (db) sqlite3_close(db);
        throw std::runtime_error("failed to open temp db");
    }

    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, "PRAGMA user_version", -1, &stmt, nullptr) != SQLITE_OK) {
        sqlite3_close(db);
        throw std::runtime_error("failed to prepare user_version");
    }
    if (sqlite3_step(stmt) != SQLITE_ROW) {
        sqlite3_finalize(stmt);
        sqlite3_close(db);
        throw std::runtime_error("failed to step user_version");
    }
    const int version = sqlite3_column_int(stmt, 0);
    sqlite3_finalize(stmt);
    sqlite3_close(db);
    return version;
}

posest::runtime::RuntimeConfig makeValidConfig() {
    posest::runtime::RuntimeConfig config;

    posest::CameraConfig cam0;
    cam0.id = "cam0";
    cam0.type = "v4l2";
    cam0.device = "/dev/video0";
    cam0.enabled = true;
    cam0.format.width = 1280;
    cam0.format.height = 720;
    cam0.format.fps = 60.0;
    cam0.format.pixel_format = "mjpeg";
    cam0.controls.push_back({"exposure_absolute", 120});
    cam0.controls.push_back({"gain", 20});
    config.cameras.push_back(cam0);

    posest::CameraConfig cam1;
    cam1.id = "cam1";
    cam1.type = "v4l2";
    cam1.device = "/dev/video1";
    cam1.enabled = false;
    config.cameras.push_back(cam1);

    posest::runtime::PipelineConfig tags;
    tags.id = "tags";
    tags.type = "apriltag";
    tags.enabled = true;
    tags.parameters_json = R"({"family":"tag36h11"})";
    config.pipelines.push_back(tags);

    posest::runtime::PipelineConfig vio;
    vio.id = "vio";
    vio.type = "vio";
    vio.enabled = false;
    vio.parameters_json = "{}";
    config.pipelines.push_back(vio);

    config.bindings.push_back({"cam0", "tags"});
    config.calibrations.push_back({"cam0", "calib/cam0.json", "v1", "2026-04-24T00:00:00Z"});
    config.camera_triggers.push_back({"cam0", true, 6, 50.0, 750, 125});
    config.teensy.serial_port = "/dev/ttyACM0";
    config.teensy.baud_rate = 921600;
    config.teensy.reconnect_interval_ms = 250;
    config.teensy.read_timeout_ms = 5;
    config.teensy.fused_pose_can_id = 0x201;
    config.teensy.status_can_id = 0x202;
    config.teensy.pose_publish_hz = 50.0;

    return config;
}

posest::CameraConfig makeCamera(const std::string& id, const std::string& device) {
    posest::CameraConfig camera;
    camera.id = id;
    camera.type = "v4l2";
    camera.device = device;
    camera.enabled = true;
    camera.format.width = 1280;
    camera.format.height = 720;
    camera.format.fps = 60.0;
    camera.format.pixel_format = "mjpeg";
    return camera;
}

std::uint32_t readU32(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    return static_cast<std::uint32_t>(bytes[offset]) |
           (static_cast<std::uint32_t>(bytes[offset + 1]) << 8u) |
           (static_cast<std::uint32_t>(bytes[offset + 2]) << 16u) |
           (static_cast<std::uint32_t>(bytes[offset + 3]) << 24u);
}

std::uint64_t readU64(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    std::uint64_t value = 0;
    for (int shift = 0; shift <= 56; shift += 8) {
        value |= static_cast<std::uint64_t>(bytes[offset + static_cast<std::size_t>(shift / 8)])
                 << static_cast<unsigned>(shift);
    }
    return value;
}

std::int64_t readI64(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    return static_cast<std::int64_t>(readU64(bytes, offset));
}

double readDouble(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    const std::uint64_t bits = readU64(bytes, offset);
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

posest::teensy::Frame decodeWrittenFrame(
    const std::vector<std::vector<std::uint8_t>>& writes,
    posest::teensy::MessageType type) {
    for (const auto& bytes : writes) {
        auto decoded = posest::teensy::decodeFrame(bytes);
        if (decoded && decoded->frame.type == type) {
            return decoded->frame;
        }
    }
    throw std::runtime_error("expected written frame was not found");
}

}  // namespace

TEST(SqliteConfigStore, CreatesDatabaseAndAppliesSchemaVersion) {
    const auto path = tempDbPath("schema");
    std::filesystem::remove(path);

    {
        posest::config::SqliteConfigStore store(path);
        EXPECT_TRUE(std::filesystem::exists(path));
    }

    EXPECT_EQ(readUserVersion(path), posest::config::currentSchemaVersion());
    std::filesystem::remove(path);
}

TEST(SqliteConfigStore, EmptyDatabaseLoadsDefaultRuntimeConfig) {
    const auto path = tempDbPath("empty");
    std::filesystem::remove(path);

    posest::config::SqliteConfigStore store(path);
    const auto config = store.loadRuntimeConfig();

    EXPECT_TRUE(config.cameras.empty());
    EXPECT_TRUE(config.pipelines.empty());
    EXPECT_TRUE(config.bindings.empty());
    EXPECT_TRUE(config.calibrations.empty());
    EXPECT_DOUBLE_EQ(config.teensy.pose_publish_hz, 50.0);
    std::filesystem::remove(path);
}

TEST(SqliteConfigStore, FullRuntimeConfigRoundTripsAndReopens) {
    const auto path = tempDbPath("roundtrip");
    std::filesystem::remove(path);
    const auto saved = makeValidConfig();

    {
        posest::config::SqliteConfigStore store(path);
        store.saveRuntimeConfig(saved);
    }

    posest::config::SqliteConfigStore reopened(path);
    const auto loaded = reopened.loadRuntimeConfig();

    ASSERT_EQ(loaded.cameras.size(), 2u);
    EXPECT_EQ(loaded.cameras[0].id, "cam0");
    EXPECT_TRUE(loaded.cameras[0].enabled);
    EXPECT_EQ(loaded.cameras[0].format.width, 1280);
    ASSERT_EQ(loaded.cameras[0].controls.size(), 2u);
    EXPECT_EQ(loaded.cameras[0].controls[0].name, "exposure_absolute");
    EXPECT_EQ(loaded.cameras[1].id, "cam1");
    EXPECT_FALSE(loaded.cameras[1].enabled);

    ASSERT_EQ(loaded.pipelines.size(), 2u);
    EXPECT_EQ(loaded.pipelines[0].id, "tags");
    EXPECT_EQ(loaded.pipelines[0].parameters_json, R"({"family":"tag36h11"})");
    EXPECT_FALSE(loaded.pipelines[1].enabled);

    ASSERT_EQ(loaded.bindings.size(), 1u);
    EXPECT_EQ(loaded.bindings[0].camera_id, "cam0");
    EXPECT_EQ(loaded.bindings[0].pipeline_id, "tags");

    ASSERT_EQ(loaded.calibrations.size(), 1u);
    EXPECT_EQ(loaded.calibrations[0].file_path, "calib/cam0.json");
    ASSERT_EQ(loaded.camera_triggers.size(), 1u);
    EXPECT_EQ(loaded.camera_triggers[0].camera_id, "cam0");
    EXPECT_TRUE(loaded.camera_triggers[0].enabled);
    EXPECT_EQ(loaded.camera_triggers[0].teensy_pin, 6);
    EXPECT_DOUBLE_EQ(loaded.camera_triggers[0].rate_hz, 50.0);
    EXPECT_EQ(loaded.camera_triggers[0].pulse_width_us, 750u);
    EXPECT_EQ(loaded.camera_triggers[0].phase_offset_us, 125);
    EXPECT_EQ(loaded.teensy.serial_port, "/dev/ttyACM0");
    EXPECT_EQ(loaded.teensy.baud_rate, 921600u);
    EXPECT_EQ(loaded.teensy.reconnect_interval_ms, 250u);
    EXPECT_EQ(loaded.teensy.read_timeout_ms, 5u);
    EXPECT_EQ(loaded.teensy.fused_pose_can_id, 0x201u);
    EXPECT_EQ(loaded.teensy.status_can_id, 0x202u);

    std::filesystem::remove(path);
}

TEST(SqliteConfigStore, FailedValidationDoesNotOverwriteExistingData) {
    const auto path = tempDbPath("atomic");
    std::filesystem::remove(path);

    posest::config::SqliteConfigStore store(path);
    store.saveRuntimeConfig(makeValidConfig());

    auto invalid = makeValidConfig();
    invalid.cameras[0].id.clear();
    EXPECT_THROW(store.saveRuntimeConfig(invalid), std::invalid_argument);

    const auto loaded = store.loadRuntimeConfig();
    ASSERT_EQ(loaded.cameras.size(), 2u);
    EXPECT_EQ(loaded.cameras[0].id, "cam0");
    std::filesystem::remove(path);
}

TEST(ConfigValidator, RejectsStrictCoreFieldFailures) {
    auto config = makeValidConfig();

    auto expect_invalid = [](posest::runtime::RuntimeConfig value) {
        EXPECT_THROW(posest::config::validateRuntimeConfig(value), std::invalid_argument);
    };

    auto duplicate_camera = config;
    duplicate_camera.cameras.push_back(duplicate_camera.cameras[0]);
    expect_invalid(duplicate_camera);

    auto invalid_format = config;
    invalid_format.cameras[0].format.width = 0;
    expect_invalid(invalid_format);

    auto duplicate_control = config;
    duplicate_control.cameras[0].controls.push_back({"gain", 99});
    expect_invalid(duplicate_control);

    auto bad_pipeline_json = config;
    bad_pipeline_json.pipelines[0].parameters_json = "[]";
    expect_invalid(bad_pipeline_json);

    auto malformed_pipeline_json = config;
    malformed_pipeline_json.pipelines[0].parameters_json = R"({"family":)";
    expect_invalid(malformed_pipeline_json);

    auto unknown_binding = config;
    unknown_binding.bindings[0].pipeline_id = "missing";
    expect_invalid(unknown_binding);

    auto disabled_binding = config;
    disabled_binding.bindings[0].camera_id = "cam1";
    expect_invalid(disabled_binding);

    auto bad_calibration = config;
    bad_calibration.calibrations[0].file_path.clear();
    expect_invalid(bad_calibration);

    auto unknown_trigger_camera = config;
    unknown_trigger_camera.camera_triggers[0].camera_id = "missing";
    expect_invalid(unknown_trigger_camera);

    auto bad_trigger_rate = config;
    bad_trigger_rate.camera_triggers[0].rate_hz = 0.0;
    expect_invalid(bad_trigger_rate);

    auto bad_trigger_pulse = config;
    bad_trigger_pulse.camera_triggers[0].pulse_width_us = 0;
    expect_invalid(bad_trigger_pulse);

    auto too_many_enabled_triggers = config;
    too_many_enabled_triggers.cameras.clear();
    too_many_enabled_triggers.camera_triggers.clear();
    for (int i = 0; i < 7; ++i) {
        const auto id = "cam" + std::to_string(i);
        too_many_enabled_triggers.cameras.push_back(makeCamera(id, "/dev/video" + std::to_string(i)));
        too_many_enabled_triggers.camera_triggers.push_back(
            {id, true, 2 + i, 50.0, 750, 0});
    }
    expect_invalid(too_many_enabled_triggers);

    auto duplicate_trigger_camera = config;
    duplicate_trigger_camera.camera_triggers.push_back(config.camera_triggers[0]);
    expect_invalid(duplicate_trigger_camera);

    auto duplicate_trigger_pin = config;
    posest::CameraConfig cam2;
    cam2.id = "cam2";
    cam2.type = "v4l2";
    cam2.device = "/dev/video2";
    duplicate_trigger_pin.cameras.push_back(cam2);
    duplicate_trigger_pin.camera_triggers.push_back({"cam2", true, 6, 30.0, 500, 0});
    expect_invalid(duplicate_trigger_pin);

    auto bad_teensy = config;
    bad_teensy.teensy.pose_publish_hz = 0.0;
    expect_invalid(bad_teensy);

    auto bad_teensy_serial_timing = config;
    bad_teensy_serial_timing.teensy.read_timeout_ms = 0;
    expect_invalid(bad_teensy_serial_timing);
}

TEST(ConfigStore, InMemoryRoundTripsRuntimeConfig) {
    posest::runtime::RuntimeConfig config;
    posest::CameraConfig camera;
    camera.id = "cam0";
    camera.type = "v4l2";
    camera.device = "/dev/video0";
    config.cameras.push_back(camera);

    posest::config::InMemoryConfigStore store;
    store.saveRuntimeConfig(config);

    const auto loaded = store.loadRuntimeConfig();
    ASSERT_EQ(loaded.cameras.size(), 1u);
    EXPECT_EQ(loaded.cameras[0].id, "cam0");
}

TEST(TeensyService, CameraTriggerConfigWireFormatMatchesFirmwareParserContract) {
    posest::MeasurementBus bus(4);
    posest::teensy::FakeSerialTransport fake;
    auto state = fake.sharedState();

    std::vector<posest::runtime::CameraTriggerConfig> triggers;
    triggers.push_back({"cam0", true, 2, 50.0, 750, 125});
    triggers.push_back({"cam1", false, 3, 30.0, 500, -25});

    posest::runtime::TeensyConfig config;
    config.serial_port = "/dev/fake-teensy";
    config.reconnect_interval_ms = 5;
    config.read_timeout_ms = 2;

    posest::teensy::TeensyService service(
        config,
        triggers,
        bus,
        [state] { return std::make_unique<posest::teensy::FakeSerialTransport>(state); });

    service.start();
    ASSERT_TRUE(fake.waitForOpenCount(1, std::chrono::milliseconds(500)));
    ASSERT_TRUE(fake.waitForWriteCount(1, std::chrono::milliseconds(500)));
    service.stop();

    const auto frame = decodeWrittenFrame(
        fake.writes(),
        posest::teensy::MessageType::ConfigCommand);
    ASSERT_EQ(frame.payload.size(), 8u + 2u * 28u);
    EXPECT_EQ(readU32(frame.payload, 0),
              static_cast<std::uint32_t>(posest::teensy::ConfigCommandKind::CameraTriggers));
    EXPECT_EQ(readU32(frame.payload, 4), 2u);

    EXPECT_EQ(readU32(frame.payload, 8), 1u);
    EXPECT_EQ(readU32(frame.payload, 12), 2u);
    EXPECT_DOUBLE_EQ(readDouble(frame.payload, 16), 50.0);
    EXPECT_EQ(readU32(frame.payload, 24), 750u);
    EXPECT_EQ(readI64(frame.payload, 28), 125);

    EXPECT_EQ(readU32(frame.payload, 36), 0u);
    EXPECT_EQ(readU32(frame.payload, 40), 3u);
    EXPECT_DOUBLE_EQ(readDouble(frame.payload, 44), 30.0);
    EXPECT_EQ(readU32(frame.payload, 52), 500u);
    EXPECT_EQ(readI64(frame.payload, 56), -25);
}
