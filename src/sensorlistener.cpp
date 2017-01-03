#include "sensorlistener.h"
#include "filters.h"

#include <boost/function.hpp>

SensorListener::SensorListener()
{
}

SensorListener::SensorListener(const Config& config)
{
    // setup sensor listeners
    for (const auto& entity : config.entities())
    {
        for (const auto& sensor : entity.sensors)
        {
            using SensorCallback = void(atlas::markerdataConstPtr);

            // data passed to the callback lambda
            SensorId sensorId;
            sensorId.entity = entity.name;
            sensorId.sensor = sensor.name;

            auto transform = sensor.transf;

            // callback lambda function
            // provides aditional values to the callback like the name of the reference frame
            boost::function<SensorCallback> callbackSensor = [this, transform, sensorId](const atlas::markerdataConstPtr markerData) {
                onSensorDataAvailable(sensorId, transform, markerData);
            };

            // tell ros we want to listen to that topic
            node.subscribe<SensorCallback>(sensor.topic, 1000, callbackSensor);
        }
    }
}

void SensorListener::onSensorDataAvailable(const SensorId& sensorId, const tf2::Transform& transform, const atlas::markerdataConstPtr markerMsg)
{
    tf2::Transform markerTransf;
    markerTransf.setOrigin({ markerMsg->position.x, markerMsg->position.y, markerMsg->position.z });
    markerTransf.setRotation({ markerMsg->orientation.x, markerMsg->orientation.y, markerMsg->orientation.z, markerMsg->orientation.w });

    // do the transformation from the sensor to the base coordinate frame of the entity
    markerTransf *= transform;

    // store it as raw data that needs filtering
    SensorData data;
    data.transf     = markerTransf;
    data.stamp      = ros::Time::now();
    data.confidence = markerMsg->confidance;
    data.id         = sensorId;

    m_rawSensorData[sensorId.entity][markerMsg->id].push_back(data);
}

SensorData SensorListener::calculateWeightedMean(const SensorDataList& data) const
{
    SensorData filteredData;

    WeightedMean meanFilter;

    if (data.empty())
        return filteredData;

    // calculate the weighted sum
    for (const auto& sensorData : data)
    {
        const double weight = std::max(0.01, sensorData.confidence);

        //std::cout << filteredData.transf.getOrigin().x() << filteredData.transf.getOrigin().y() << filteredData.transf.getOrigin().z() << std::endl;

        meanFilter.addVec3(sensorData.transf.getOrigin(), weight);
        meanFilter.addQuat(sensorData.transf.getRotation(), weight);
    }

    // calculate the weighted average
    filteredData.transf.setOrigin(meanFilter.weightedMeanVec3());
    filteredData.transf.setRotation(meanFilter.weightedMeanQuat());

    return filteredData;
}

FilteredSensorDataMap SensorListener::filteredSensorData() const
{
    FilteredSensorDataMap filteredSensorData;

    // calculate a weighted average over the sensor data
    for (const auto& keyval : m_rawSensorData)
    {
        const auto& entityName = keyval.first;
        const auto& markerData = keyval.second;

        for (const auto& keyval : markerData)
        {
            const auto& markerId   = keyval.first;
            const auto& sensorData = keyval.second;

            filteredSensorData[entityName][markerId] = calculateWeightedMean(sensorData);
        }
    }

    return filteredSensorData;
}

SensorDataMap SensorListener::rawSensorData() const
{
    return m_rawSensorData;
}
