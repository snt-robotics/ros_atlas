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
            broadcast("world", entityName, pose);

            if (publishMarkers)
            {
                for (const auto& marker : m_markers[entityName])
                    broadcast(entityName, "Marker " + std::to_string(marker.id), marker.transf);
            }

            if (publishEntitySensors)
            {
                for (const auto& sensor : m_entitySensors[entityName])
                    broadcast(entityName, sensor.name, sensor.transf);
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
            broadcast("world", sensor.name, sensor.transf);
    }
}

void TransformGraphBroadcaster::broadcast(const std::string& frame, const std::string& child, const tf2::Transform transf)
{
    geometry_msgs::TransformStamped transform;
    transform.header.stamp    = ros::Time::now();
    transform.header.frame_id = frame;
    transform.child_frame_id  = child;

    transform.transform.rotation.x = transf.getRotation().x();
    transform.transform.rotation.y = transf.getRotation().y();
    transform.transform.rotation.z = transf.getRotation().z();
    transform.transform.rotation.w = transf.getRotation().w();

    transform.transform.translation.x = transf.getOrigin().x();
    transform.transform.translation.y = transf.getOrigin().y();
    transform.transform.translation.z = transf.getOrigin().z();

    m_tfbc.sendTransform(transform);
}

void TransformGraphBroadcaster::broadcast(const std::string& frame, const std::string& child, const Pose pose)
{
    geometry_msgs::TransformStamped transform;
    transform.header.stamp    = ros::Time::now();
    transform.header.frame_id = frame;
    transform.child_frame_id  = child;

    transform.transform.rotation.x = pose.rot.x();
    transform.transform.rotation.y = pose.rot.y();
    transform.transform.rotation.z = pose.rot.z();
    transform.transform.rotation.w = pose.rot.w();

    transform.transform.translation.x = pose.pos.x();
    transform.transform.translation.y = pose.pos.y();
    transform.transform.translation.z = pose.pos.z();

    m_tfbc.sendTransform(transform);
}
