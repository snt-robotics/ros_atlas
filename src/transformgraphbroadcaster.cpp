#include "transformgraphbroadcaster.h"

#include <geometry_msgs/TransformStamped.h>

TransformGraphBroadcaster::TransformGraphBroadcaster(const Config& config)
{
    // load data from config
    for (const auto& marker : config.markers())
        m_markers[marker.ref].push_back(marker);

    for (const auto& entity : config.entities())
        m_entitySensors[entity.name] = entity.sensors;

    m_worldSensors = config.worldSensors();
}

void TransformGraphBroadcaster::broadcast(const TransformGraph& graph, bool publishMarkers, bool publishEntitySensors, bool publishWorldSensors)
{
    auto entityNames = graph.entities();
    for (const auto& entityName : entityNames)
    {
        Pose pose;
        try
        {
            pose = graph.lookupPose(entityName);

            // compose geometry message
            geometry_msgs::TransformStamped transform;
            transform.header.stamp    = ros::Time::now();
            transform.header.frame_id = "world"; // the lookup is always in world coordinates
            transform.child_frame_id  = entityName;

            transform.transform.rotation.x = pose.rot.x();
            transform.transform.rotation.y = pose.rot.y();
            transform.transform.rotation.z = pose.rot.z();
            transform.transform.rotation.w = pose.rot.w();

            transform.transform.translation.x = pose.pos.x();
            transform.transform.translation.y = pose.pos.y();
            transform.transform.translation.z = pose.pos.z();

            m_tfbc.sendTransform(transform);

            if (publishMarkers)
            {
                for (const auto& marker : m_markers[entityName])
                {
                    geometry_msgs::TransformStamped transform;
                    transform.header.stamp    = ros::Time::now();
                    transform.header.frame_id = entityName;
                    transform.child_frame_id  = "Marker " + std::to_string(marker.id);

                    transform.transform.rotation.x = marker.transf.getRotation().x();
                    transform.transform.rotation.y = marker.transf.getRotation().y();
                    transform.transform.rotation.z = marker.transf.getRotation().z();
                    transform.transform.rotation.w = marker.transf.getRotation().w();

                    transform.transform.translation.x = marker.transf.getOrigin().x();
                    transform.transform.translation.y = marker.transf.getOrigin().y();
                    transform.transform.translation.z = marker.transf.getOrigin().z();

                    m_tfbc.sendTransform(transform);
                }
            }

            if (publishEntitySensors)
            {
                for (const auto& sensor : m_entitySensors[entityName])
                {
                    geometry_msgs::TransformStamped transform;
                    transform.header.stamp    = ros::Time::now();
                    transform.header.frame_id = entityName;
                    transform.child_frame_id  = sensor.name;

                    transform.transform.rotation.x = sensor.transf.getRotation().x();
                    transform.transform.rotation.y = sensor.transf.getRotation().y();
                    transform.transform.rotation.z = sensor.transf.getRotation().z();
                    transform.transform.rotation.w = sensor.transf.getRotation().w();

                    transform.transform.translation.x = sensor.transf.getOrigin().x();
                    transform.transform.translation.y = sensor.transf.getOrigin().y();
                    transform.transform.translation.z = sensor.transf.getOrigin().z();

                    m_tfbc.sendTransform(transform);
                }
            }
        }
        catch (const std::string&)
        {
            // no lookup possible
        }
    }

    if (publishWorldSensors)
    {
        for (const auto& sensor : m_worldSensors)
        {
            geometry_msgs::TransformStamped transform;
            transform.header.stamp    = ros::Time::now();
            transform.header.frame_id = "world";
            transform.child_frame_id  = sensor.name;

            transform.transform.rotation.x = sensor.transf.getRotation().x();
            transform.transform.rotation.y = sensor.transf.getRotation().y();
            transform.transform.rotation.z = sensor.transf.getRotation().z();
            transform.transform.rotation.w = sensor.transf.getRotation().w();

            transform.transform.translation.x = sensor.transf.getOrigin().x();
            transform.transform.translation.y = sensor.transf.getOrigin().y();
            transform.transform.translation.z = sensor.transf.getOrigin().z();

            m_tfbc.sendTransform(transform);
        }
    }
}
