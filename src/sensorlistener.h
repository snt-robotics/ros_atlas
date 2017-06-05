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

#include "config.h"
#include "filters.h"
#include <atlas/MarkerData.h>
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
     * @param sigma: The standard deviation
     */
    Measurement(const Key& key, const tf2::Vector3& pos, const tf2::Quaternion& rot, double sigma = 1.0)
        : transform(rot, pos)
        , stamp(ros::Time::now())
        , key(key)
        , sigma(sigma)
    {
    }

    /**
     * @brief SensorData
     * @param sensorId: The sensor the data is coming from
     * @param pos: The position of the entity
     * @param sigma: The standard deviation
     */
    Measurement(const Key& key, const tf2::Vector3& pos, double sigma = 1.0)
        : transform(tf2::Quaternion::getIdentity(), pos)
        , stamp(ros::Time::now())
        , key(key)
        , sigma(sigma)
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

    /// The pose of the entity
    tf2::Transform transform;

    /// The time this data was recorded
    ros::Time stamp;

    /// The unique identifier for this measurement
    Key key;

    /// The standard deviation
    double sigma = 1.0;
};

using SensorDataList        = std::vector<Measurement>;
using SensorDataMap         = std::map<Measurement::Key, ExplonentialMovingAverageFilter>;
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

    /**
     * @brief SensorListener
     * @param config: Used to configure the sensor listener
     */
    SensorListener(const Config& config);

    /**
     * @brief filteredSensorData
     * @return A weighted average of all sensor measurements
     */
    SensorDataList filteredSensorData() const;

    /**
     * @brief clear clears all recorded sensor data
     */
    void clear();

    /**
     * @brief onSensorDataAvailable is the callback used by ROS in case new data is available
     * @param from: Where the data origins from
     * @param sensor: The name of the sensor
     * @param sensorTransform: The transformation from the sensor to the baselink of "from"
     * @param markerMsg: The marker message
     */
    void onSensorDataAvailable(const std::string& from, const std::string& to, const std::string& sensor, const tf2::Transform& sensorTransform, const tf2::Transform& entityMarkerTransform, const atlas::MarkerData& markerMsg);

protected:
    void setupMarkerBasedSensor(const Entity& entity, const Sensor& sensor);
    void setupNonMarkerBasedSensor(const Entity& entity, const Sensor& sensor);

private:
    ros::NodeHandle m_node;
    std::vector<ros::Subscriber> m_subscribers;

    // used to map from the marker id to the target entity
    std::map<int, std::pair<std::string, Marker> > m_markers;

    // sensor data
    SensorDataMap m_rawSensorData;
};
