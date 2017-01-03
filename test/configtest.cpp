#include "helpers.h"

#include "../src/config.h"

const std::string yamlInput = "entities:\n"
                              "  - entity:\n"
                              "    name: 'ardrone1'\n"
                              "    sensors:\n"
                              "      - sensor:\n"
                              "        name: 'fontcam'\n"
                              "        topic: 'aruco/ardrone1/etc'\n"
                              "        transform:\n"
                              "          origin: [1, 0, 0]\n"
                              "          rot: [0, 0, 0, 1]\n"
                              "\n"
                              "  - entity:\n"
                              "    name: 'ardrone2'\n"
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
                              "";

TEST(Config, basic)
{
    Config config;

    config.loadFromString(yamlInput);

    ASSERT_EQ(2, config.entities().size());
    ASSERT_EQ(2, config.markers().size());

    ASSERT_EQ("ardrone1", config.entities()[0].name);
    ASSERT_EQ("ardrone2", config.entities()[1].name);

    ASSERT_EQ(1, config.markers()[0].id);
    ASSERT_EQ(4, config.markers()[1].id);
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
