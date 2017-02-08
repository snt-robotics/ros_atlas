#pragma once

#include <string>
#include <tf2/LinearMath/Transform.h>
#include <yaml-cpp/node/node.h>

/**
 * @brief The Sensor struct
 * @var name is the name of sensor and its coordinate frame
 * @var topic must comply to the definition
 * @var transf denotes the transformation from the sensor frame to the baselink
 */
struct Sensor
{
    std::string name;
    std::string topic;
    tf2::Transform transf = tf2::Transform::getIdentity();
};

/**
 * @brief The WorldSensor struct
 * @var name: Name of the sensor
 * @var entity: Entity name which the sensor is measuring
 * @var topic: The ROS topic of type stamped pose
 * @var sigma: The standrad deviation
 */
struct WorldSensor
{
    enum class Type
    {
        None, ///< unspecified
        Global, ///< Non marker based (e.g. GPS)
        Camera ///< Camera, marker based
    };
    std::string name;
    std::string entity;
    std::string topic;
    tf2::Transform transf = tf2::Transform::getIdentity(); ///< transform to world frame
    Type type             = Type::None;
    double sigma          = 1.0;
    int fakeId            = -1;
};

/**
 * @brief The Entity struct
 * An entity is a generic named object
 * The name also defines the baselink of the object
 * An entity may have an arbitrary amount of sensors
 */
struct Entity
{
    std::string name;
    std::vector<Sensor> sensors;
};

/**
 * @brief The Marker struct
 * Defines a marker by id and reference frame
 * @var transf denotes the transformation from the marker to the
 * origin of the frame it is defined in
 */
struct Marker
{
    int id = -1;
    std::string ref;
    tf2::Transform transf;
};

/**
 * @brief The Options struct
 */
struct Options
{
    std::string dbgGraphFilename; ///< The file to save the graph to
    double dbgGraphInterval   = 0.0; ///< The graph saving interval in seconds
    double loopRate           = 60.0; ///< Loop rate of the node in Hz
    double decayDuration      = 0.25; ///< Decay time of the graph's edges in seconds
    bool publishMarkers       = true; ///< Publishes the markers via the ros tf system
    bool publishWorldSensors  = true; ///< Publishes the world sensors via the ros tf system
    bool publishEntitySensors = true; ///< Publishes the entity sensors via the ros tf system
};

class Config
{
public:
    /**
     * @brief Config contains all of the configuration data specified by the user
     * @param filename is the YAML config file to load
     */
    Config(const std::string& filename);

    /**
     * @brief Config
     * Creates an empty config
     * Use loadFromString to populate
     */
    Config();

    void loadFromString(const std::string& input);

    /**
     * @brief entities
     * @return the entities in the config file
     */
    std::vector<Entity> entities() const;

    /**
     * @brief markers
     * @return the markers defined in the config file
     */
    std::vector<Marker> markers() const;

    /**
     * @brief worldSensors
     * @return the world sensors as specified in the config file
     */
    std::vector<WorldSensor> worldSensors() const;

    /**
     * @brief options
     * @return the options as specified in the config file
     */
    Options options() const;

    /**
     * @brief dump prints the current configuration
     */
    void dump() const;

protected:
    /**
     * @brief parseTransform creates a ros transform from a YAML node
     * @param node contains the YAML node describing a transformation
     * @return a ros transform
     */
    tf2::Transform parseTransform(const YAML::Node& node) const;

    void parseRoot(const YAML::Node& node);

private:
    std::vector<Entity> m_entities;
    std::vector<Marker> m_markers;
    std::vector<WorldSensor> m_worldSensors;
    Options m_options;
};
