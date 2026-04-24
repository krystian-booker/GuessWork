#include <string>

#include <gtest/gtest.h>

#include "posest/config/InMemoryConfigStore.h"
#include "posest/config/SqliteSchema.h"

TEST(ConfigSchema, ContainsRequiredRuntimeTables) {
    const std::string schema = posest::config::sqliteSchemaSql();

    EXPECT_NE(schema.find("CREATE TABLE IF NOT EXISTS cameras"), std::string::npos);
    EXPECT_NE(schema.find("CREATE TABLE IF NOT EXISTS camera_controls"), std::string::npos);
    EXPECT_NE(schema.find("CREATE TABLE IF NOT EXISTS pipelines"), std::string::npos);
    EXPECT_NE(schema.find("CREATE TABLE IF NOT EXISTS camera_pipeline_bindings"), std::string::npos);
    EXPECT_NE(schema.find("CREATE TABLE IF NOT EXISTS calibrations"), std::string::npos);
    EXPECT_NE(schema.find("CREATE TABLE IF NOT EXISTS teensy_config"), std::string::npos);
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
