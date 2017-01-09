#include "sensorlistener.h"
#include "filters.h"

#include <boost/function.hpp>
#include <geometry_msgs/PoseStamped.h>

SensorListener::SensorListener()
{
}

SensorListener::SensorListener(const Config& config)
{
    // setup marker sensor listeners
    for (const auto& entity : config.entities())
    {
        for (const auto& sensor : entity.sensors)
        {
            using SensorCallback = void(atlas::MarkerDataConstPtr);

            // data passed to the callback lambda
            auto from       = entity.name;
            auto sensorName = sensor.name;
            auto transform  = sensor.transf;

            // callback lambda function
            // provides aditional values to the callback like the name of the reference frame
            boost::function<SensorCallback> callbackSensor = [this, transform, from, sensorName](const atlas::MarkerDataConstPtr markerData) {
                onSensorDataAvailable(from, sensorName, transform, *markerData);
            };

            // tell ros we want to listen to that topic
            node.subscribe<SensorCallback>(sensor.topic, 1000, callbackSensor);
        }
    }

    // setup world sensor listeners
    for (const auto& sensor : config.worldSensors())
    {
        if (sensor.type == WorldSensor::Type::GPS)
        {
            using SensorCallback = void(geometry_msgs::PoseStampedConstPtr);

            // data passed to the callback lambda
            auto to = sensor.entity;

            // callback lambda function
            // provides aditional values to the callback like the name of the reference frame
            boost::function<SensorCallback> callbackSensor = [this, to](const geometry_msgs::PoseStampedConstPtr data) {
                atlas::MarkerData fakeData;
                fakeData.pos = data->pose.position;
                fakeData.rot = data->pose.orientation;

                onSensorDataAvailable("world", to, tf2::Transform::getIdentity(), fakeData);
            };

            // tell ros we want to listen to that topic
            node.subscribe<SensorCallback>(sensor.topic, 1000, callbackSensor);
        }
        else // marker based callback
        {
            using SensorCallback = void(atlas::MarkerDataConstPtr);

            // data passed to the callback lambda
            auto from       = "world";
            auto sensorName = sensor.name;
            auto transform  = sensor.transf;

            // callback lambda function
            // provides aditional values to the callback like the name of the reference frame
            boost::function<SensorCallback> callbackSensor = [this, transform, from, sensorName](const atlas::MarkerDataConstPtr markerData) {
                onSensorDataAvailable(from, sensorName, transform, *markerData);
            };

            // tell ros we want to listen to that topic
            node.subscribe<SensorCallback>(sensor.topic, 1000, callbackSensor);
        }
    }

    // setup markers
    // contains the information to map a marker to an entity
    for (const auto& marker : config.markers())
    {
        m_markers[marker.id] = marker;
    }
}

void SensorListener::onSensorDataAvailable(const std::string& from, const std::string& sensor, const tf2::Transform& sensorTransform, const atlas::MarkerData& markerMsg)
{
    // store the transformation of the marker in the sensor frame
    tf2::Transform markerTransf;
    markerTransf.setOrigin({ markerMsg.pos.x, markerMsg.pos.y, markerMsg.pos.z });
    markerTransf.setRotation({ markerMsg.rot.x, markerMsg.rot.y, markerMsg.rot.z, markerMsg.rot.w });

    // do the transformation from the marker pose to the base of the entity it is attached to
    markerTransf *= m_markers[markerMsg.id].transf;

    // do the transformation from base of the marker to the base of the entity the sensor is attached to
    markerTransf *= sensorTransform;

    // store it as raw data that needs filtering
    Measurement measurement;
    measurement.transform  = markerTransf;
    measurement.stamp      = ros::Time::now();
    measurement.sigma      = markerMsg.sigma;
    measurement.key.from   = from;
    measurement.key.to     = m_markers[markerMsg.id].ref;
    measurement.key.sensor = sensor;
    measurement.key.marker = markerMsg.id;

    m_rawSensorData[measurement.key].push_back(measurement);
}

Measurement SensorListener::calculateWeightedMean(const SensorDataList& data) const
{
    Measurement filteredData;

    WeightedMean meanFilter;

    if (data.empty())
        return filteredData;

    // calculate the weighted sum
    for (const auto& sensorData : data)
    {
        const double weight = std::max(0.01, sensorData.sigma);

        //std::cout << filteredData.transf.getOrigin().x() << filteredData.transf.getOrigin().y() << filteredData.transf.getOrigin().z() << std::endl;

        meanFilter.addVec3(sensorData.transform.getOrigin(), weight);
        meanFilter.addQuat(sensorData.transform.getRotation(), weight);
    }

    // calculate the weighted average
    filteredData.transform.setOrigin(meanFilter.weightedMeanVec3());
    filteredData.transform.setRotation(meanFilter.weightedMeanQuat());

    return filteredData;
}

SensorDataList SensorListener::filteredSensorData() const
{
    SensorDataList filteredSensorData;

    // calculate a weighted average over the sensor data
    for (const auto& keyval : m_rawSensorData)
    {
        const auto& measurements = keyval.second;
        filteredSensorData.push_back(calculateWeightedMean(measurements));
    }

    return filteredSensorData;
}

void SensorListener::clear()
{
    m_rawSensorData.clear();
}
