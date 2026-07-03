#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

#include "server/camera_repository.hpp"
#include "server/config_snapshot.hpp"
#include "server/database.hpp"
#include "server/field_layout_repository.hpp"
#include "server/fusion_config_repository.hpp"
#include "server/imu_config_repository.hpp"
#include "server/net_config_repository.hpp"
#include "server/trigger_group_repository.hpp"
#include "server/vio_config_repository.hpp"

namespace gw::server {

namespace {

// Minimal but VALID WPILib layouts — import runs parse_field_layout_json,
// which rejects empty tag arrays.
constexpr const char* kLayoutJsonA =
    R"({"field": {"length": 16.541, "width": 8.069},
        "tags": [{"ID": 1, "pose": {"translation": {"x": 1.0, "y": 2.0, "z": 1.3},
                  "rotation": {"quaternion": {"W": 1.0, "X": 0.0, "Y": 0.0, "Z": 0.0}}}}]})";
constexpr const char* kLayoutJsonB =
    R"({"field": {"length": 17.0, "width": 8.0},
        "tags": [{"ID": 2, "pose": {"translation": {"x": 3.0, "y": 1.0, "z": 1.0},
                  "rotation": {"quaternion": {"W": 0.0, "X": 0.0, "Y": 0.0, "Z": 1.0}}}}]})";

// Both repositories bundles around one temp DB.
struct Repos {
    explicit Repos(Database& db)
        : cameras(db), trigger_groups(db), field_layouts(db), imu_config(db),
          vio_config(db), net_config(db), fusion_config(db) {}

    CameraRepository       cameras;
    TriggerGroupRepository trigger_groups;
    FieldLayoutRepository  field_layouts;
    ImuConfigRepository    imu_config;
    VioConfigRepository    vio_config;
    NetConfigRepository    net_config;
    FusionConfigRepository fusion_config;

    crow::json::wvalue export_all() {
        return export_snapshot(cameras, trigger_groups, field_layouts,
                               imu_config, vio_config, net_config,
                               fusion_config);
    }
    ImportReport import_all(const crow::json::rvalue& snap) {
        return import_snapshot(snap, cameras, trigger_groups, field_layouts,
                               imu_config, vio_config, net_config,
                               fusion_config);
    }
};

class ConfigSnapshotTest : public ::testing::Test {
protected:
    void SetUp() override {
        const std::string base =
            "gw_snapshot_test_" + std::to_string(::getpid()) + "_" +
            ::testing::UnitTest::GetInstance()->current_test_info()->name();
        path_a_ = std::filesystem::temp_directory_path() / (base + "_a.db");
        path_b_ = std::filesystem::temp_directory_path() / (base + "_b.db");
        Database::remove_files(path_a_);
        Database::remove_files(path_b_);
        db_a_ = std::make_unique<Database>(path_a_);
        db_b_ = std::make_unique<Database>(path_b_);
        a_    = std::make_unique<Repos>(*db_a_);
        b_    = std::make_unique<Repos>(*db_b_);
    }
    void TearDown() override {
        a_.reset();
        b_.reset();
        db_a_.reset();
        db_b_.reset();
        Database::remove_files(path_a_);
        Database::remove_files(path_b_);
    }

    // Populates DB A with a representative robot config.
    void populate_a() {
        const auto cam1 = a_->cameras.create("front", "SER-1", 6.0,
                                             std::nullopt, true, 3);
        CameraUpdate u1;
        u1.role     = std::optional<std::string>{"apriltag"};
        u1.gain     = 5.5;
        u1.exposure = 2000.0;
        a_->cameras.update(cam1.id, u1);
        a_->cameras.set_calibration(cam1.id, "camchain-yaml-blob");
        a_->cameras.set_imu_extrinsics(cam1.id, "imucam-yaml-blob");
        a_->cameras.create("rear", "SER-2", 8.0);

        a_->trigger_groups.create("apriltag", 30.0, {1, 2});
        a_->trigger_groups.create("vio", 30.0, {3, 4});

        a_->field_layouts.create("season", kLayoutJsonA);  // auto-active
        const auto practice = a_->field_layouts.create("practice", kLayoutJsonB);
        a_->field_layouts.activate(practice.id);

        ImuConfigUpdate imu;
        imu.gyro_noise_density = 3.3e-4;
        imu.t_imu_robot_json   = std::optional<std::string>{
            R"({"T_robot_imu": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]})"};
        a_->imu_config.update(imu);

        VioConfigUpdate vio;
        vio.num_pts = 200;
        a_->vio_config.update(vio);

        NetConfigUpdate net;  // every field off its default
        net.enabled    = false;
        net.bind_port  = 5801;
        net.robot_port = 5802;
        net.robot_ip   = "10.28.52.2";
        a_->net_config.update(net);

        FusionConfigUpdate fusion;
        fusion.lag_s     = 3.0;
        fusion.output_hz = 50;
        a_->fusion_config.update(fusion);
    }

    // Snapshot comparison ignores timestamp-bearing fields.
    static std::string normalized(crow::json::wvalue snap) {
        auto r = crow::json::load(snap.dump());
        crow::json::wvalue out(r);
        out["exported_at"] = 0;
        // calibrated_at / extrinsics_calibrated_at are restamped on import.
        if (r.has("cameras")) {
            crow::json::wvalue::list cams;
            for (const auto& c : r["cameras"]) {
                crow::json::wvalue cj(c);
                if (c.has("calibrated_at") &&
                    c["calibrated_at"].t() != crow::json::type::Null) {
                    cj["calibrated_at"] = 0;
                }
                if (c.has("extrinsics_calibrated_at") &&
                    c["extrinsics_calibrated_at"].t() != crow::json::type::Null) {
                    cj["extrinsics_calibrated_at"] = 0;
                }
                cams.emplace_back(std::move(cj));
            }
            out["cameras"] = std::move(cams);
        }
        return out.dump();
    }

    std::filesystem::path     path_a_, path_b_;
    std::unique_ptr<Database> db_a_, db_b_;
    std::unique_ptr<Repos>    a_, b_;
};

}  // namespace

TEST_F(ConfigSnapshotTest, RoundTripRestoresEverything) {
    populate_a();
    // B starts as a "fresh install" with a seeded layout (merge exercise).
    b_->field_layouts.seed_if_empty("season", kLayoutJsonA);

    const auto snap_a = a_->export_all();
    const auto parsed = crow::json::load(snap_a.dump());
    const auto report = b_->import_all(parsed);

    EXPECT_TRUE(report.ok()) << [&] {
        std::string all;
        for (const auto& e : report.cameras.errors) all += e + "; ";
        for (const auto& e : report.trigger_groups.errors) all += e + "; ";
        for (const auto& e : report.field_layouts.errors) all += e + "; ";
        return all;
    }();
    EXPECT_EQ(report.cameras.created, 2);
    EXPECT_EQ(report.field_layouts.created, 1);  // "practice"
    EXPECT_EQ(report.field_layouts.updated, 1);  // "season" (same json)

    EXPECT_EQ(normalized(a_->export_all()), normalized(b_->export_all()));

    // Spot-check the expensive bits made it.
    const auto cam = b_->cameras.find_by_serial("SER-1");
    ASSERT_TRUE(cam.has_value());
    EXPECT_EQ(cam->calibration_json, "camchain-yaml-blob");
    EXPECT_EQ(cam->imu_extrinsics_json, "imucam-yaml-blob");
    EXPECT_EQ(cam->trigger_output_pin, 3);
    ASSERT_TRUE(b_->field_layouts.get_active().has_value());
    EXPECT_EQ(b_->field_layouts.get_active()->name, "practice");
    const auto net = b_->net_config.get();
    EXPECT_FALSE(net.enabled);
    EXPECT_EQ(net.bind_port, 5801);
    EXPECT_EQ(net.robot_port, 5802);
    EXPECT_EQ(net.robot_ip, "10.28.52.2");
    EXPECT_DOUBLE_EQ(b_->fusion_config.get().lag_s, 3.0);
}

TEST_F(ConfigSnapshotTest, MergeIsNonDestructive) {
    populate_a();
    b_->cameras.create("extra", "SER-99", 4.0);
    b_->trigger_groups.create("extra-group", 60.0, {6});

    const auto parsed =
        crow::json::load(a_->export_all().dump());
    const auto report = b_->import_all(parsed);
    EXPECT_TRUE(report.ok());

    EXPECT_TRUE(b_->cameras.find_by_serial("SER-99").has_value());
    bool extra_group = false;
    for (const auto& g : b_->trigger_groups.list_all()) {
        if (g.name == "extra-group") extra_group = true;
    }
    EXPECT_TRUE(extra_group);
}

TEST_F(ConfigSnapshotTest, PinSwapResolves) {
    populate_a();
    // B has the two snapshot cameras with the pin on the OTHER camera.
    b_->cameras.create("front", "SER-1", 6.0);
    b_->cameras.create("rear", "SER-2", 8.0, std::nullopt, true, 3);

    const auto parsed =
        crow::json::load(a_->export_all().dump());
    const auto report = b_->import_all(parsed);
    EXPECT_TRUE(report.ok());

    EXPECT_EQ(b_->cameras.find_by_serial("SER-1")->trigger_output_pin, 3);
    EXPECT_FALSE(
        b_->cameras.find_by_serial("SER-2")->trigger_output_pin.has_value());
}

TEST_F(ConfigSnapshotTest, PinHeldByLocalCameraReportsError) {
    populate_a();
    // A non-snapshot camera holds pin 3 — never stolen.
    b_->cameras.create("local-only", "SER-LOCAL", 4.0, std::nullopt, true, 3);

    const auto parsed =
        crow::json::load(a_->export_all().dump());
    const auto report = b_->import_all(parsed);

    EXPECT_FALSE(report.ok());
    ASSERT_FALSE(report.cameras.errors.empty());
    EXPECT_NE(report.cameras.errors[0].find("SER-1"), std::string::npos);
    // The conflicting camera failed; the other one still imported.
    EXPECT_TRUE(b_->cameras.find_by_serial("SER-2").has_value());
    EXPECT_EQ(b_->cameras.find_by_serial("SER-LOCAL")->trigger_output_pin, 3);
}

TEST_F(ConfigSnapshotTest, NameCollisionReportsAndSkipsBlobs) {
    populate_a();
    // B: different serial already owns the name "front".
    b_->cameras.create("front", "SER-OTHER", 4.0);

    const auto parsed =
        crow::json::load(a_->export_all().dump());
    const auto report = b_->import_all(parsed);

    EXPECT_FALSE(report.ok());
    // SER-1 was created (serial unknown) but its update to name "front"
    // collided — error recorded; its blobs were skipped.
    bool found = false;
    for (const auto& e : report.cameras.errors) {
        if (e.find("SER-1") != std::string::npos) found = true;
    }
    EXPECT_TRUE(found);
}

TEST_F(ConfigSnapshotTest, ActiveLayoutReplacedInPlace) {
    // Fresh-install case: B's single seeded ACTIVE layout shares the
    // snapshot's name but carries different json.
    a_->field_layouts.create("season", kLayoutJsonB);  // active in A
    b_->field_layouts.seed_if_empty("season", kLayoutJsonA);

    const auto parsed =
        crow::json::load(a_->export_all().dump());
    const auto report = b_->import_all(parsed);
    EXPECT_TRUE(report.ok());

    const auto active = b_->field_layouts.get_active();
    ASSERT_TRUE(active.has_value());
    EXPECT_EQ(active->name, "season");
    EXPECT_EQ(active->json, kLayoutJsonB);
}

TEST_F(ConfigSnapshotTest, BadSectionFailsPartially) {
    populate_a();
    auto snap = a_->export_all();
    snap["net_config"]["bind_port"] = 80;  // CHECK violation on import

    const auto parsed = crow::json::load(snap.dump());
    const auto report = b_->import_all(parsed);

    EXPECT_FALSE(report.ok());
    EXPECT_FALSE(report.net_config.errors.empty());
    // Everything else still applied.
    EXPECT_TRUE(report.cameras.errors.empty());
    // The failed UPDATE is atomic — the whole row keeps its defaults.
    EXPECT_TRUE(b_->net_config.get().enabled);
    EXPECT_EQ(b_->net_config.get().bind_port, 5809);
    EXPECT_DOUBLE_EQ(b_->fusion_config.get().lag_s, 3.0);
}

TEST_F(ConfigSnapshotTest, LegacyCanConfigSectionIgnored) {
    // Pre-UDP exports carried a "can_config" section (the Teensy CAN bridge,
    // since replaced by the UDP robot link). Like any unknown section it must
    // be silently skipped — never an import failure.
    crow::json::wvalue snap;
    snap["snapshot_version"]       = kSnapshotVersion;
    snap["can_config"]["mode"]     = "roborio";
    snap["fusion_config"]["lag_s"] = 4.0;

    const auto parsed = crow::json::load(snap.dump());
    const auto report = b_->import_all(parsed);

    EXPECT_TRUE(report.ok());
    EXPECT_TRUE(report.net_config.errors.empty());
    EXPECT_EQ(report.net_config.updated, 0);
    // net_config row untouched (schema defaults).
    const auto net = b_->net_config.get();
    EXPECT_TRUE(net.enabled);
    EXPECT_EQ(net.bind_port, 5809);
    EXPECT_EQ(net.robot_port, 5810);
    EXPECT_EQ(net.robot_ip, "");
    // Recognized sections in the same snapshot still applied.
    EXPECT_DOUBLE_EQ(b_->fusion_config.get().lag_s, 4.0);
}

TEST_F(ConfigSnapshotTest, VersionMismatchThrows) {
    crow::json::wvalue bad;
    bad["snapshot_version"] = 99;
    const auto parsed = crow::json::load(bad.dump());
    EXPECT_THROW(b_->import_all(parsed), std::runtime_error);
}

}  // namespace gw::server
