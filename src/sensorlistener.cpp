#include "sensorlistener.h"
#include "filters.h"
#include "helpers.h"

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
            if (sensor.type == Sensor::Type::MarkerBased)
            {
                setupMarkerBasedSensor(entity, sensor);
            }
            else
            {
                setupNonMarkerBasedSensor(entity, sensor);
            }
        }

        // setup markers
        // contains the information to map a marker to an entity
        for (const auto& marker : entity.markers)
        {
            m_markers[marker.id] = { entity.name, marker };
        }
    }
}

void SensorListener::onSensorDataAvailable(const std::string& from, const std::string& sensor, const tf2::Transform& sensorTransform, const atlas::MarkerData& markerMsg)
{
    // store the transformation of the marker in the sensor space
    tf2::Transform markerTransf;
    markerTransf.setOrigin({ markerMsg.pos.x, markerMsg.pos.y, markerMsg.pos.z });
    markerTransf.setRotation({ markerMsg.rot.x, markerMsg.rot.y, markerMsg.rot.z, markerMsg.rot.w });

    // calculate the transform
    tf2::Transform transf = sensorTransform * markerTransf * m_markers[markerMsg.id].second.transf.inverse();

    // store it as raw data that needs filtering
    Measurement measurement;
    measurement.transform  = transf;
    measurement.stamp      = ros::Time::now();
    measurement.sigma      = markerMsg.sigma;
    measurement.key.from   = from;
    measurement.key.to     = m_markers[markerMsg.id].first;
    measurement.key.sensor = sensor;
    measurement.key.marker = markerMsg.id;

    // filter
    // setup
    m_rawSensorData[measurement.key].setTimeout(ros::Duration(0.25));

    // add the new data to the filter
    m_rawSensorData[measurement.key].addQuat(measurement.transform.getRotation());
    m_rawSensorData[measurement.key].addVec3(measurement.transform.getOrigin());
    m_rawSensorData[measurement.key].addScalar(measurement.sigma);
}

void SensorListener::setupMarkerBasedSensor(const Entity& entity, const Sensor& sensor)
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
    m_subscribers.push_back(m_node.subscribe<SensorCallback>(sensor.topic, 1000, callbackSensor));
    ROS_INFO("Suscribed to topic \"%s\"", sensor.topic.c_str());
}

void SensorListener::setupNonMarkerBasedSensor(const Entity& entity, const Sensor& sensor)
{
    using SensorCallback = void(geometry_msgs::PoseStampedConstPtr);

    // data passed to the callback lambda
    auto sensorName = sensor.name;
    auto id         = sensor.fakeId;
    auto sigma      = sensor.sigma;

    // callback lambda function
    // provides aditional values to the callback like the name of the reference frame
    boost::function<SensorCallback> callbackSensor = [this, sensorName, id, sigma](const geometry_msgs::PoseStampedConstPtr data) {

        // This marker does not exist.
        // Its sole purpose is to map the sensor
        // readings to an entity.
        atlas::MarkerData fakeData;
        fakeData.pos   = data->pose.position;
        fakeData.rot   = data->pose.orientation;
        fakeData.id    = id;
        fakeData.sigma = sigma;

        onSensorDataAvailable("world", sensorName, tf2::Transform::getIdentity(), fakeData);
    };

    // tell ros we want to listen to that topic
    m_subscribers.push_back(m_node.subscribe<SensorCallback>(sensor.topic, 1000, callbackSensor));
    ROS_INFO("Suscribed to topic \"%s\"", sensor.topic.c_str());
}

SensorDataList SensorListener::filteredSensorData() const
{
    SensorDataList filteredSensorData;

    // calculate a weighted average over the sensor data
    for (const auto& keyval : m_rawSensorData)
    {
        const auto& filter = keyval.second;

        Measurement filteredData;
        filteredData.key   = keyval.first;
        filteredData.stamp = ros::Time::now();
        filteredData.transform.setOrigin(filter.vec3());
        filteredData.transform.setRotation(filter.quat());
        filteredData.sigma = filter.scalar();

        filteredSensorData.push_back(filteredData);
    }

    return filteredSensorData;
}

void SensorListener::clear()
{
    m_rawSensorData.clear();
}
