#pragma once

#include "config.h"
#include <atlas/markerdata.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>

/**
 * @brief The SensorData struct
 */
struct Measurement
{
    struct Key
    {
        Key(const std::string& from, const std::string& to, const std::string& sensor, int marker)
            : from(from)
            , to(to)
            , sensor(sensor)
            , marker(marker)
        {
        }

        Key() {}

        std::string from;
        std::string to;
        std::string sensor;
        int marker = -1;

        // operators needed by std::map
        bool operator<(const Key& other) const
        {
            return std::tie(from, to, sensor, marker) < std::tie(other.from, other.to, other.sensor, other.marker);
        }

        bool operator==(const Key& other) const
        {
            return std::tie(from, to, sensor, marker) == std::tie(other.from, other.to, other.sensor, other.marker);
        }
    };

    /**
     * @brief SensorData
     * @param sensorId: The sensor the data is coming from
     * @param pos: The position of the entity
     * @param rot: The rotation of the entity
     */
    Measurement(const Key& key, const tf2::Vector3& pos, const tf2::Quaternion& rot)
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
    Measurement(const Key& key, const tf2::Vector3& pos)
        : transform(tf2::Quaternion::getIdentity(), pos)
        , stamp(ros::Time::now())
        , key(key)
    {
    }

    /**
     * @brief SensorData
     * @param sensorId: The sensor the data is coming from
     */
    Measurement(const Key& key)
        : transform(tf2::Quaternion::getIdentity(), { 0, 0, 0 })
        , stamp(ros::Time::now())
        , key(key)
    {
    }

    Measurement() {}

    // pose
    tf2::Transform transform;

    // todo
    double confidence = 0.0;

    // the time this data was recorded
    ros::Time stamp;

    // a unique identifier for this data
    Key key;
};

using SensorDataList        = std::vector<Measurement>;
using SensorDataMap         = std::map<Measurement::Key, std::vector<Measurement> >;
using FilteredSensorDataMap = std::map<std::string, std::map<int, Measurement> >;

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

    SensorDataList filteredSensorData() const;

    void onSensorDataAvailable(const std::string& from, const std::string& sensor, const tf2::Transform& sensorTransform, const atlas::markerdataConstPtr markerMsg);

protected:
    Measurement calculateWeightedMean(const SensorDataList& data) const;

private:
    ros::NodeHandle node;

    std::map<int, Marker> m_markers;

    // sensor data
    SensorDataMap m_rawSensorData;
    //std::map<SensorDataKey, SensorData> m_filteredSensorData;
};
