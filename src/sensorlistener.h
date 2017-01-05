#pragma once

#include "config.h"
#include <atlas/markerdata.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>

struct MeasurementKey
{
    MeasurementKey(const std::string& entity, const std::string& sensor, int marker)
        : entity(entity)
        , sensor(sensor)
        , marker(marker)
    {
    }

    MeasurementKey() {}

    std::string entity;
    std::string sensor;
    int marker = -1;

    bool operator<(const MeasurementKey& other) const
    {
        return std::tie(entity, sensor, marker) < std::tie(other.entity, other.sensor, other.marker);
    }

    bool operator==(const MeasurementKey& other) const
    {
        return std::tie(entity, sensor, marker) == std::tie(other.entity, other.sensor, other.marker);
    }
};

struct SensorData
{
    /**
     * @brief SensorData
     * @param sensorId: The sensor the data is coming from
     * @param pos: The position of the entity
     * @param rot: The rotation of the entity
     */
    SensorData(const MeasurementKey& key, const tf2::Vector3& pos, const tf2::Quaternion& rot)
        : transform(rot, pos)
        , stamp(ros::Time::now())
        , key(key)
    {
    }

    /**
     * @brief SensorData
     * @param sensorId: The sensor the data is coming from
     * @param pos: The position of the entity
     */
    SensorData(const MeasurementKey& key, const tf2::Vector3& pos)
        : transform(tf2::Quaternion::getIdentity(), pos)
        , stamp(ros::Time::now())
        , key(key)
    {
    }

    /**
     * @brief SensorData
     * @param sensorId: The sensor the data is coming from
     */
    SensorData(const MeasurementKey& key)
        : transform(tf2::Quaternion::getIdentity(), { 0, 0, 0 })
        , stamp(ros::Time::now())
        , key(key)
    {
    }

    SensorData() {}

    // pose
    tf2::Transform transform;

    double confidence = 0.0;
    ros::Time stamp;

    MeasurementKey key;
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

    void onSensorDataAvailable(const MeasurementKey& measurementKey, const tf2::Transform& sensorTransform, const atlas::markerdataConstPtr markerMsg);

protected:
    SensorData calculateWeightedMean(const SensorDataList& data) const;

private:
    ros::NodeHandle node;

    std::map<int, Marker> m_markers;

    // sensor data
    SensorDataMap m_rawSensorData;
    //std::map<SensorDataKey, SensorData> m_filteredSensorData;
};
