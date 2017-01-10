#include "helpers.h"

#include "../src/config.h"

const std::string yamlInput = "entities:\n"
                              "  - entity:\n"
                              "    name: 'ardrone0'\n"
                              "    sensors:\n"
                              "      - sensor:\n"
                              "        name: 'fontcam'\n"
                              "        topic: 'aruco/ardrone1/etc'\n"
                              "        transform:\n"
                              "          origin: [1, 0, 0]\n"
                              "          rot: [0, 0, 0, 1]\n"
                              "\n"
                              "  - entity:\n"
                              "    name: 'ardrone1'\n"
                              "\n"
                              "markers:\n"
                              "  - marker:\n"
                              "    id: 1\n"
                              "    ref: 'world'\n"
                              "    transform:\n"
                              "      origin: [1, 0, 0]\n"
                              "      rot: [0, 0, 0, 1]\n"
                              "\n"
                              "  - marker:\n"
                              "    id: 4\n"
                              "    ref: 'ardrone1'\n"
                              "    transform:\n"
                              "      origin: [1, 0, 0]\n"
                              "      rot: [0, 0, 0, 1]\n"
                              "\n"
                              "world:\n"
                              "  - sensor:\n"
                              "    name: 'optitrack0'\n"
                              "    type: 'Global'\n"
                              "    topic: '/Ardrone2SimpleLinModel_HASHMARK_0/pose'\n"
                              "    entity: 'ardrone0'\n"
                              "    sigma: 0.5\n"
                              "\n"
                              "  - sensor:\n"
                              "    name: 'optitrack1'\n"
                              "    type: 'Global'\n"
                              "    topic: '/Ardrone2SimpleLinModel_HASHMARK_1/pose'\n"
                              "    entity: 'ardrone1'\n"
                              "    sigma: 0.5";

TEST(Config, basic)
{
    Config config;

    config.loadFromString(yamlInput);

    ASSERT_EQ(2, config.entities().size());
    ASSERT_EQ(4, config.markers().size());

    // entity name check
    ASSERT_EQ("ardrone0", config.entities()[0].name);
    ASSERT_EQ("ardrone1", config.entities()[1].name);

    // marker id check
    ASSERT_EQ(1, config.markers()[0].id);
    ASSERT_EQ(4, config.markers()[1].id);

    // mocap fake markers check
    ASSERT_EQ(-1, config.markers()[2].id);
    ASSERT_EQ(-2, config.markers()[3].id);
}

TEST(Config, transform)
{
    Config config;

    config.loadFromString(yamlInput);

    tf2::Transform transf;
    transf.setOrigin({ 1, 0, 0 });
    transf.setRotation({ 0, 0, 0, 1 });

    ASSERT_EQ(1, config.markers()[0].id);
    ASSERT_TRUE(transfEq(transf, config.markers()[0].transf));
}

TEST(Config, ref)
{
    Config config;

    config.loadFromString(yamlInput);

    ASSERT_EQ("world", config.markers()[0].ref);
}
