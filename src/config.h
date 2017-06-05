/*
 * ATLAS - Cooperative sensing
 * Copyright (C) 2017  Paul KREMER
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <string>
#include <tf2/LinearMath/Transform.h>
#include <yaml-cpp/node/node.h>

/**
 * @brief The Sensor struct
 */
struct Sensor
{
    enum class Type
    {
        /// Marker based sensors (e.g. cameras)
        /// Expects topics of type MarkerData
        /// (default)
        MarkerBased,

        /// Non marker based sensors (e.g. GPS, optitrack)
        /// Expects topics of type PoseStamped
        NonMarkerBased
    };
    std::string name; ///< the name of sensor and its coordinate frame
    std::string topic; ///< the ROS topic. Message type depends on sensor type.
    std::string target; ///< target of the sensor (NonMarkerBased)
    tf2::Transform transf = tf2::Transform::getIdentity(); ///< transform to world frame
    Type type             = Type::MarkerBased; ///< the type of the sensor (see Type)
    double sigma          = 1.0; ///< only used if not provided by the topic
};

/**
 * @brief The Marker struct
 */
struct Marker
{
    int id = -1; ///< id of the marker. Valid IDs are > 0.
    tf2::Transform transf; ///< transformation of the marker relative to its parent
};

/**
 * @brief The FilterConfig struct
 */
struct FilterConfig
{
    double alpha = 0.1; ///< exp. moving average constant
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
    std::vector<Marker> markers;
    FilterConfig filterConfig;
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
    bool publishPoseTopics    = true; ///< Publishes the fused poses as topics of type PoseStamped
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
    Options m_options;
};
