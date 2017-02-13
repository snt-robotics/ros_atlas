#include "helpers.h"

#include "../src/config.h"

const std::string yamlInput = //
    "---\n"
    "options:\n"
    "  # node settings\n"
    "  loopRate: 1.0 # Hz\n"
    "\n"
    "  # graph settings\n"
    "  decayDuration: 1.0 # seconds\n"
    "\n"
    "  # transform publisher settings\n"
    "  publishMarkers: false\n"
    "  publishWorldSensors: false\n"
    "  publishEntitySensors: false\n"
    "\n"
    "  # debug\n"
    "  dbgDumpGraphInterval: 1.0 # seconds\n"
    "  dbgDumpGraphFilename: 'dbgGraph.dot'\n"
    "\n"
    "entities:\n"
    "  - entity: world\n"
    "    sensors:\n"
    "    - sensor: optitrack0\n"
    "      topic: '/Ardrone2SimpleLinModel_HASHMARK_0/pose'\n"
    "      type: 'NonMarkerBased'\n"
    "      target: ardrone0\n"
    "      sigma: 0.1\n"
    "\n"
    "    - sensor: optitrack1\n"
    "      topic: '/Ardrone2SimpleLinModel_HASHMARK_1/pose'\n"
    "      target: ardrone1\n"
    "      type: 'NonMarkerBased'\n"
    "      sigma: 0.1\n"
    "\n"
    "  - entity: ardrone0\n"
    "    sensors:\n"
    "    - sensor: fontcam\n"
    "      topic: '/aruco_tracker/ardrone0_frontcam/detected_markers'\n"
    "      transform: {origin: [1, 2, 3], rot: [0, 0, 0, 1]}\n"
    "\n"
    "  - entity: ardrone1\n"
    "    sensors:\n"
    "    - sensor: fontcam\n"
    "      topic: '/aruco_tracker/ardrone1_frontcam/detected_markers'\n"
    "      transform: {origin: [4, 5, 6], rot: [0, 0, 0, 1]}\n"
    "\n"
    "  # This entity is used to detect the position of marker 0\n"
    "  # It has no physical meaning\n"
    "  - entity: Marker0Wrapper\n"
    "    markers:\n"
    "    - marker: 0\n"
    "      transform: {origin: [7, 8, 9], rot: [1, 0, 1, 0]}\n"
    "...\n"
    "";

TEST(Config, options)
{
    Config config;
    config.loadFromString(yamlInput);

    ASSERT_EQ("dbgGraph.dot", config.options().dbgGraphFilename);
    ASSERT_EQ(1.0, config.options().dbgGraphInterval);
    ASSERT_EQ(false, config.options().publishEntitySensors);
    ASSERT_EQ(false, config.options().publishWorldSensors);
    ASSERT_EQ(1.0, config.options().decayDuration);
    ASSERT_EQ(1.0, config.options().decayDuration);
    ASSERT_EQ(1.0, config.options().loopRate);
}

TEST(Config, entities)
{
    Config config;
    config.loadFromString(yamlInput);

    // size check
    ASSERT_EQ(4, config.entities().size()); // world, ardrone0, ardrone1, Marker0Wrapper
    ASSERT_EQ(1, config.entities()[3].markers.size());

    // entity name check
    ASSERT_EQ("world", config.entities()[0].name);
    ASSERT_EQ("ardrone0", config.entities()[1].name);
    ASSERT_EQ("ardrone1", config.entities()[2].name);
    ASSERT_EQ("Marker0Wrapper", config.entities()[3].name);

    // marker id check
    ASSERT_EQ(0, config.entities()[3].markers[0].id);

    // sensor check
    ASSERT_EQ("optitrack0", config.entities()[0].sensors[0].name);
    ASSERT_EQ("optitrack1", config.entities()[0].sensors[1].name);
    ASSERT_EQ("fontcam", config.entities()[1].sensors[0].name);
    ASSERT_EQ("fontcam", config.entities()[2].sensors[0].name);

    // sigma check
    ASSERT_EQ(0.1, config.entities()[0].sensors[0].sigma);
    ASSERT_EQ(0.1, config.entities()[0].sensors[1].sigma);

    // target check
    ASSERT_EQ("ardrone0", config.entities()[0].sensors[0].target);
    ASSERT_EQ("ardrone1", config.entities()[0].sensors[1].target);
}

TEST(Config, transform)
{
    Config config;
    config.loadFromString(yamlInput);

    tf2::Transform transf;
    transf.setOrigin({ 7, 8, 9 });
    transf.setRotation({ 1, 0, 1, 0 });

    ASSERT_TRUE(transfEq(transf, config.entities()[3].markers[0].transf));
}
