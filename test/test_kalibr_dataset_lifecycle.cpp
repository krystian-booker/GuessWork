#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "posest/IFrameProducer.h"
#include "posest/config/SqliteConfigStore.h"
#include "posest/runtime/Daemon.h"
#include "posest/runtime/Factories.h"

namespace {

std::filesystem::path tempDbPath(const std::string& name) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() /
           ("posest_lifecycle_" + name + "_" + std::to_string(stamp) + ".db");
}

std::filesystem::path tempDirPath(const std::string& name) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    auto dir = std::filesystem::temp_directory_path() /
               ("posest_lifecycle_" + name + "_" + std::to_string(stamp));
    std::filesystem::create_directories(dir);
    // Drop a sentinel file inside so remove_all has something to clean up.
    std::ofstream(dir / "sentinel.txt") << "lifecycle test artifact\n";
    return dir;
}

// runConfigCommand needs a camera factory by reference; the list / delete
// dispatch paths never invoke it, but we have to satisfy the signature.
class UnusedCameraFactory final : public posest::runtime::ICameraBackendFactory {
public:
    std::shared_ptr<posest::IFrameProducer> createCamera(
        const posest::CameraConfig& /*config*/) override {
        throw std::runtime_error(
            "camera factory should not be invoked by list/delete dispatch");
    }
};

// RAII redirect for std::cout. Restores the original streambuf even on
// exception.
class StdoutCapture final {
public:
    StdoutCapture() : prior_(std::cout.rdbuf(captured_.rdbuf())) {}
    ~StdoutCapture() { std::cout.rdbuf(prior_); }
    StdoutCapture(const StdoutCapture&) = delete;
    StdoutCapture& operator=(const StdoutCapture&) = delete;

    std::string str() const { return captured_.str(); }

private:
    std::stringstream captured_;
    std::streambuf* prior_;
};

posest::runtime::RuntimeConfig configWithDatasets(
    std::vector<posest::runtime::KalibrDatasetConfig> datasets) {
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
    config.cameras.push_back(cam0);
    config.kalibr_datasets = std::move(datasets);
    return config;
}

posest::runtime::DaemonOptions listOptions(
    const std::filesystem::path& db_path, bool json) {
    posest::runtime::DaemonOptions options;
    options.command = posest::runtime::DaemonCommand::ListKalibrDatasets;
    options.config_path = db_path;
    options.list_kalibr_datasets.json = json;
    return options;
}

posest::runtime::DaemonOptions deleteOptions(
    const std::filesystem::path& db_path,
    std::string id,
    bool remove_files) {
    posest::runtime::DaemonOptions options;
    options.command = posest::runtime::DaemonCommand::DeleteKalibrDataset;
    options.config_path = db_path;
    options.delete_kalibr_dataset.id = std::move(id);
    options.delete_kalibr_dataset.remove_files = remove_files;
    return options;
}

}  // namespace

TEST(KalibrDatasetLifecycle, ListEmitsRowsAsJsonAndText) {
    const auto db_path = tempDbPath("list");
    std::filesystem::remove(db_path);
    const auto on_disk_dir = tempDirPath("on_disk");
    const std::string missing_path = "/tmp/posest_lifecycle_does_not_exist";

    {
        posest::config::SqliteConfigStore store(db_path);
        store.saveRuntimeConfig(configWithDatasets({
            {on_disk_dir.string(), on_disk_dir.string(),
             "2026-04-27T00:00:00Z", 12.5, {"cam0"}},
            {missing_path, missing_path,
             "2026-04-26T00:00:00Z", 5.0, {"cam0"}},
        }));
    }

    auto store = std::make_unique<posest::config::SqliteConfigStore>(db_path);
    UnusedCameraFactory factory;

    {
        StdoutCapture capture;
        posest::runtime::runConfigCommand(
            listOptions(db_path, /*json=*/true), *store, factory);
        const auto json = nlohmann::json::parse(capture.str());
        ASSERT_TRUE(json.is_array());
        ASSERT_EQ(json.size(), 2u);
        // SqliteConfigStore loads rows ordered by id; "/tmp/..." sorts before
        // the temp_directory_path-based one on Linux only sometimes — assert
        // by lookup rather than index.
        bool saw_on_disk = false, saw_missing = false;
        for (const auto& entry : json) {
            if (entry.at("id").get<std::string>() == on_disk_dir.string()) {
                EXPECT_TRUE(entry.at("exists_on_disk").get<bool>());
                EXPECT_DOUBLE_EQ(entry.at("duration_s").get<double>(), 12.5);
                EXPECT_EQ(entry.at("camera_ids").size(), 1u);
                saw_on_disk = true;
            } else if (entry.at("id").get<std::string>() == missing_path) {
                EXPECT_FALSE(entry.at("exists_on_disk").get<bool>());
                saw_missing = true;
            }
        }
        EXPECT_TRUE(saw_on_disk);
        EXPECT_TRUE(saw_missing);
    }

    {
        StdoutCapture capture;
        posest::runtime::runConfigCommand(
            listOptions(db_path, /*json=*/false), *store, factory);
        const auto out = capture.str();
        EXPECT_NE(out.find(on_disk_dir.string()), std::string::npos);
        EXPECT_NE(out.find(missing_path), std::string::npos);
        EXPECT_NE(out.find("on_disk"), std::string::npos);
        EXPECT_NE(out.find("missing"), std::string::npos);
        EXPECT_NE(out.find("cameras=cam0"), std::string::npos);
    }

    std::filesystem::remove_all(on_disk_dir);
    std::filesystem::remove(db_path);
}

TEST(KalibrDatasetLifecycle, DeleteRemovesRowOnly) {
    const auto db_path = tempDbPath("delete_row_only");
    std::filesystem::remove(db_path);
    const auto dir = tempDirPath("delete_row_only");

    {
        posest::config::SqliteConfigStore store(db_path);
        store.saveRuntimeConfig(configWithDatasets({
            {dir.string(), dir.string(),
             "2026-04-27T00:00:00Z", 5.0, {"cam0"}},
        }));
    }

    posest::config::SqliteConfigStore store(db_path);
    UnusedCameraFactory factory;
    posest::runtime::runConfigCommand(
        deleteOptions(db_path, dir.string(), /*remove_files=*/false),
        store, factory);

    const auto loaded = store.loadRuntimeConfig();
    EXPECT_TRUE(loaded.kalibr_datasets.empty());
    EXPECT_TRUE(std::filesystem::is_directory(dir));

    std::filesystem::remove_all(dir);
    std::filesystem::remove(db_path);
}

TEST(KalibrDatasetLifecycle, DeleteRemovesRowAndDirectory) {
    const auto db_path = tempDbPath("delete_with_files");
    std::filesystem::remove(db_path);
    const auto dir = tempDirPath("delete_with_files");

    {
        posest::config::SqliteConfigStore store(db_path);
        store.saveRuntimeConfig(configWithDatasets({
            {dir.string(), dir.string(),
             "2026-04-27T00:00:00Z", 5.0, {"cam0"}},
        }));
    }

    posest::config::SqliteConfigStore store(db_path);
    UnusedCameraFactory factory;
    posest::runtime::runConfigCommand(
        deleteOptions(db_path, dir.string(), /*remove_files=*/true),
        store, factory);

    const auto loaded = store.loadRuntimeConfig();
    EXPECT_TRUE(loaded.kalibr_datasets.empty());
    EXPECT_FALSE(std::filesystem::exists(dir));

    std::filesystem::remove(db_path);
}

TEST(KalibrDatasetLifecycle, DeleteToleratesMissingDirectory) {
    const auto db_path = tempDbPath("delete_missing_dir");
    std::filesystem::remove(db_path);
    const std::string missing_path = "/tmp/posest_lifecycle_already_gone";

    {
        posest::config::SqliteConfigStore store(db_path);
        store.saveRuntimeConfig(configWithDatasets({
            {missing_path, missing_path,
             "2026-04-27T00:00:00Z", 5.0, {"cam0"}},
        }));
    }

    posest::config::SqliteConfigStore store(db_path);
    UnusedCameraFactory factory;
    EXPECT_NO_THROW(posest::runtime::runConfigCommand(
        deleteOptions(db_path, missing_path, /*remove_files=*/true),
        store, factory));

    const auto loaded = store.loadRuntimeConfig();
    EXPECT_TRUE(loaded.kalibr_datasets.empty());

    std::filesystem::remove(db_path);
}

TEST(KalibrDatasetLifecycle, DeleteThrowsOnUnknownId) {
    const auto db_path = tempDbPath("delete_unknown");
    std::filesystem::remove(db_path);

    {
        posest::config::SqliteConfigStore store(db_path);
        // Empty config: no kalibr_datasets at all.
        store.saveRuntimeConfig(configWithDatasets({}));
    }

    posest::config::SqliteConfigStore store(db_path);
    UnusedCameraFactory factory;
    EXPECT_THROW(
        posest::runtime::runConfigCommand(
            deleteOptions(db_path, "no_such_id", /*remove_files=*/false),
            store, factory),
        std::invalid_argument);

    std::filesystem::remove(db_path);
}
