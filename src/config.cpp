#include "config.h"

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

Config::Config(const std::string& filename)
{
    auto root = YAML::LoadFile(filename);
    parseRoot(root);
}

Config::Config()
{
}

void Config::loadFromString(const std::string& input)
{
    auto root = YAML::Load(input);
    parseRoot(root);
}

tf2::Transform Config::parseTransform(const YAML::Node& node) const
{
    if (node.IsNull())
        return tf2::Transform::getIdentity();

    // parse rotation
    tf2::Quaternion rot = tf2::Quaternion::getIdentity();
    if (node["rot"].size() == 4)
    {
        rot.setX(node["rot"][0].as<double>());
        rot.setY(node["rot"][1].as<double>());
        rot.setZ(node["rot"][2].as<double>());
        rot.setW(node["rot"][3].as<double>());
    }
    else if (node["rot"].size() == 3)
    {
        rot.setRPY(node["rot"][0].as<double>(), node["rot"][1].as<double>(), node["rot"][2].as<double>());
    }
    else
    {
        ROS_WARN_ONCE("Config: 'rot' is expected to have either 3 elements (YPR) or 4 elements (quaternion). Default is {0,0,0,1}");
    }

    // parse position
    tf2::Vector3 origin;
    if (node["origin"].size() == 3)
    {
        origin.setX(node["origin"][0].as<double>());
        origin.setY(node["origin"][1].as<double>());
        origin.setZ(node["origin"][2].as<double>());
    }
    else
    {
        ROS_WARN_ONCE("Config: 'origin' is expected to have 3 elements. Default is {0,0,0}");
    }

    return tf2::Transform(rot, origin);
}

void Config::parseRoot(const YAML::Node& node)
{
    if (!node)
        ROS_ERROR("Config: Document is empty");

    // quick sanity checks
    if (!node["entities"])
        ROS_ERROR("Config: Cannot find 'entities'");

    if (!node["markers"])
        ROS_ERROR("Config: Cannot find 'markers'");

    // load the entities
    for (const YAML::Node& entity : node["entities"])
    {
        Entity entityData;
        entityData.name = entity["name"].as<std::string>();

        // load the sensor data
        if (!entity["sensors"].IsNull())
        {
            for (const auto& sensor : entity["sensors"])
            {
                Sensor sensorData;
                sensorData.name   = sensor["name"].as<std::string>();
                sensorData.topic  = sensor["topic"].as<std::string>();
                sensorData.transf = parseTransform(sensor["transform"]);

                entityData.sensors.push_back(sensorData);
            }
        }

        m_entities.push_back(entityData);
    }

    // load the markers
    for (const auto& marker : node["markers"])
    {
        Marker markerData;
        markerData.id     = marker["id"].as<int>();
        markerData.ref    = marker["ref"].as<std::string>();
        markerData.transf = parseTransform(marker["transform"]);

        m_markers.push_back(markerData);
    }

    // load the world sensors
    int markerId = -1;
    std::map<std::string, WorldSensor::Type> typeMap = {
        { "Global", WorldSensor::Type::Global },
        { "Camera", WorldSensor::Type::Camera }
    };

    for (const auto& sensor : node["world"])
    {
        WorldSensor worldSensor;
        worldSensor.entity = sensor["entity"].as<std::string>();
        worldSensor.topic  = sensor["topic"].as<std::string>();
        worldSensor.name   = sensor["name"].as<std::string>();
        worldSensor.type   = typeMap[sensor["type"].as<std::string>()];
        worldSensor.sigma  = sensor["sigma"].as<double>();
        worldSensor.transf = parseTransform(sensor["transform"]);

        if (worldSensor.type != WorldSensor::Type::None)
        {
            // Motion capture needs fake markers (ID < 0) as they do not actually detect markers
            if (worldSensor.type == WorldSensor::Type::Global)
            {
                worldSensor.fakeId = markerId;

                Marker fakeMarker;
                fakeMarker.id     = markerId;
                fakeMarker.ref    = worldSensor.entity;
                fakeMarker.transf = tf2::Transform::getIdentity();

                m_markers.push_back(fakeMarker);

                markerId--;
            }

            m_worldSensors.push_back(worldSensor);
        }
    }
}

std::vector<WorldSensor> Config::worldSensors() const
{
    return m_worldSensors;
}

std::vector<Marker> Config::markers() const
{
    return m_markers;
}

std::vector<Entity> Config::entities() const
{
    return m_entities;
}

void Config::dump() const
{
    std::cout << "\n=== CONFIG ===\n";
    std::cout << "Entities:\n";
    for (const auto& entity : m_entities)
    {
        std::cout << "  -" << entity.name << "\n";
        std::cout << "    Sensors:\n";

        for (const auto& sensor : entity.sensors)
        {
            std::cout << "      -" << sensor.name << "\n";
        }
    }

    std::cout << "Markers:\n";
    for (const auto& marker : m_markers)
    {
        std::cout << "  -ID: " << marker.id << "\n";
        std::cout << "    Ref. frame: " << marker.ref << "\n";
    }

    std::cout << "World:\n";
    for (const auto& sensor : m_worldSensors)
    {
        std::cout << "  -: " << sensor.name << "\n";
        std::cout << "    Target: " << sensor.entity << "\n";
    }

    std::cout << "=== CONFIG END ===\n";
    std::cout << std::endl;
}
