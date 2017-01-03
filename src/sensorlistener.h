#pragma once

#include "config.h"
#include <atlas/markerdata.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>

struct SensorId
{
    SensorId(const std::string& entity, const std::string& sensor)
        : entity(entity)
        , sensor(sensor)
    {
    }

    SensorId() {}

    std::string entity;
    std::string sensor;
};

struct SensorData
{
    SensorData(const SensorId& sensorId, const tf2::Transform& transform)
        : transf(transform)
        , stamp(ros::Time::now())
        , id(sensorId)
    {
    }

    SensorData() {}

    tf2::Transform transf;
    double confidence = 0.0;
    ros::Time stamp;

    SensorId id;
};

using SensorDataList        = std::vector<SensorData>;
using SensorDataMap         = std::map<std::string, std::map<int, SensorDataList> >;
using FilteredSensorDataMap = std::map<std::string, std::map<int, SensorData> >;

/**
 * @brief The SensorListener class
 * Listens to topics of type SensorData
 * Calculates the marker positions in the corresp. sensor's entity baselink frame
 */
class SensorListener
{
public:
    SensorListener();
    SensorListener(const Config& config);

    SensorDataMap rawSensorData() const;
    FilteredSensorDataMap filteredSensorData() const;

    void onSensorDataAvailable(const SensorId& sensorId, const tf2::Transform& transform, const atlas::markerdataConstPtr markerMsg);

protected:
    SensorData calculateWeightedMean(const SensorDataList& data) const;

private:
    ros::NodeHandle node;

    // sensor data
    SensorDataMap m_rawSensorData;
    //std::map<SensorDataKey, SensorData> m_filteredSensorData;
};
