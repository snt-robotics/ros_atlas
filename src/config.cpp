#include "config.h"

#include <angles/angles.h>
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
    if (!node)
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
        rot.setRPY( //
            angles::from_degrees(node["rot"][0].as<double>()), //
            angles::from_degrees(node["rot"][1].as<double>()), //
            angles::from_degrees(node["rot"][2].as<double>()) //
            );
    }
    else if (node["rot"].size() != 0)
    {
        ROS_WARN("Config: 'rot' is expected to have either 3 elements (YPR) or 4 elements (quaternion), got %i. Default is {0,0,0,1}",
            int(node["rot"].size()));
    }

    // parse position
    tf2::Vector3 origin;
    if (node["origin"].size() == 3)
    {
        origin.setX(node["origin"][0].as<double>());
        origin.setY(node["origin"][1].as<double>());
        origin.setZ(node["origin"][2].as<double>());
    }
    else if (node["origin"].size() != 0)
    {
        ROS_WARN("Config: 'origin' is expected to have 3 elements, got %i. Default is {0,0,0}",
            int(node["origin"].size()));
    }

    return tf2::Transform(rot, origin);
}

void Config::parseRoot(const YAML::Node& node)
{
    // fake marker counter
    int markerId = 0;

    // sensor type conversion
    std::map<std::string, Sensor::Type> typeMap = {
        { "MarkerBased", Sensor::Type::MarkerBased },
        { "NonMarkerBased", Sensor::Type::NonMarkerBased }
    };

    if (!node)
        ROS_ERROR("Config: Document is empty");

    // quick sanity checks
    if (!node["entities"])
        ROS_WARN("Config: Cannot find 'entities'");
    if (!node["options"])
        ROS_WARN("Config: Cannot find 'options'");

    std::map<std::string, std::vector<Marker> > fakeMarkers;

    // load the entities
    for (const YAML::Node& entity : node["entities"])
    {
        Entity entityData;
        entityData.name = entity["name"].as<std::string>("undefined");

        // load the sensor data
        for (const auto& sensor : entity["sensors"])
        {
            Sensor sensorData;
            sensorData.name   = sensor["name"].as<std::string>("undefined");
            sensorData.topic  = sensor["topic"].as<std::string>("undefined");
            sensorData.transf = parseTransform(sensor["transform"]);
            sensorData.type   = typeMap[sensor["type"].as<std::string>("MarkerBased")];
            sensorData.sigma  = sensor["sigma"].as<double>(1.0);

            // Non marker based sensors use fake markers (ID < 0) as they do not actually detect markers
            if (sensorData.type == Sensor::Type::NonMarkerBased)
            {
                const auto targetEntity = sensor["target"].as<std::string>("undefined");

                // assign the fake id
                sensorData.fakeId = --markerId;

                Marker fakeMarker;
                fakeMarker.id     = markerId;
                fakeMarker.transf = tf2::Transform::getIdentity();

                // add the fake marker
                fakeMarkers[targetEntity].push_back(fakeMarker);
            }

            // add the sensor
            entityData.sensors.push_back(sensorData);
        }

        // load the markers
        for (const auto& marker : entity["markers"])
        {
            Marker markerData;
            markerData.id     = marker["id"].as<int>(-1);
            markerData.transf = parseTransform(marker["transform"]);

            entityData.markers.push_back(markerData);
        }

        // add the entity
        m_entities.push_back(entityData);
    }

    // assign the fake markers to the entities
    for (auto& entity : m_entities)
    {
        entity.markers.insert(entity.markers.end(),
            fakeMarkers[entity.name].begin(),
            fakeMarkers[entity.name].end());
    }

    // load the options
    const auto options = node["options"];

    if (options)
    {
        if (options["dbgDumpGraphFilename"])
            m_options.dbgGraphFilename = options["dbgDumpGraphFilename"].as<std::string>("");

        if (options["dbgDumpGraphInterval"])
            m_options.dbgGraphInterval = options["dbgDumpGraphInterval"].as<double>(0);

        if (options["loopRate"])
            m_options.loopRate = options["loopRate"].as<double>(60.0);

        if (options["decayDuration"])
            m_options.decayDuration = options["decayDuration"].as<double>(0.25);

        if (options["publishMarkers"])
            m_options.publishMarkers = options["publishMarkers"].as<bool>(true);

        if (options["publishWorldSensors"])
            m_options.publishWorldSensors = options["publishWorldSensors"].as<bool>(true);

        if (options["publishEntitySensors"])
            m_options.publishEntitySensors = options["publishEntitySensors"].as<bool>(true);
    }
}

Options Config::options() const
{
    return m_options;
}

std::vector<Entity> Config::entities() const
{
    return m_entities;
}

void Config::dump() const
{
    std::cout << "\n=== CONFIG ===\n";

    std::cout << "Options:\n";
    std::cout << "  loopRate: " << m_options.loopRate << "\n";
    std::cout << "  decayDuration: " << m_options.decayDuration << "\n";
    std::cout << "  dbgGraphFilename: " << m_options.dbgGraphFilename << "\n";
    std::cout << "  dbgGraphInterval: " << m_options.dbgGraphInterval << "\n";
    std::cout << "  publishMarkers: " << m_options.publishMarkers << "\n";
    std::cout << "  publishWorldSensors: " << m_options.publishWorldSensors << "\n";
    std::cout << "  publishEntitySensors: " << m_options.publishEntitySensors << "\n";

    std::cout << "Entities:\n";
    for (const auto& entity : m_entities)
    {
        std::cout << "  -" << entity.name << "\n";
        std::cout << "    Sensors:\n";

        for (const auto& sensor : entity.sensors)
        {
            std::cout << "      -" << sensor.name << "\n";
        }

        std::cout << "    Markers:\n";

        for (const auto& marker : entity.markers)
        {
            std::cout << "      -ID:" << marker.id << "\n";
        }
    }

    std::cout << "=== CONFIG END ===\n";
    std::cout << std::endl;
}
