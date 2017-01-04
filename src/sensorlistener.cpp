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
            MeasurementKey key;
            key.entity = entity.name;
            key.sensor = sensor.name;

            auto transform = sensor.transf;

            // callback lambda function
            // provides aditional values to the callback like the name of the reference frame
            boost::function<SensorCallback> callbackSensor = [this, transform, key](const atlas::markerdataConstPtr markerData) {
                onSensorDataAvailable(key, transform, markerData);
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

void SensorListener::onSensorDataAvailable(const MeasurementKey& measurementKey, const tf2::Transform& sensorTransform, const atlas::markerdataConstPtr markerMsg)
{
    // store the transformation of the marker in the sensor frame
    tf2::Transform markerTransf;
    markerTransf.setOrigin({ markerMsg->position.x, markerMsg->position.y, markerMsg->position.z });
    markerTransf.setRotation({ markerMsg->orientation.x, markerMsg->orientation.y, markerMsg->orientation.z, markerMsg->orientation.w });

    // do the transformation from the marker pose to the base of the entity it is attached to
    markerTransf *= m_markers[markerMsg->id].transf;

    // do the transformation from base of the marker to the base of the entity the sensor is attached to
    markerTransf *= sensorTransform;

    // store it as raw data that needs filtering
    SensorData data;
    data.transform  = markerTransf;
    data.stamp      = ros::Time::now();
    data.confidence = markerMsg->confidance;
    data.key        = measurementKey;
    data.key.marker = markerMsg->id;

    m_rawSensorData[measurementKey.entity][markerMsg->id].push_back(data);
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

        meanFilter.addVec3(sensorData.transform.getOrigin(), weight);
        meanFilter.addQuat(sensorData.transform.getRotation(), weight);
    }

    // calculate the weighted average
    filteredData.transform.setOrigin(meanFilter.weightedMeanVec3());
    filteredData.transform.setRotation(meanFilter.weightedMeanQuat());

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
