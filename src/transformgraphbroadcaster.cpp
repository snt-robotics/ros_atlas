#include "transformgraphbroadcaster.h"

#include <geometry_msgs/TransformStamped.h>

TransformGraphBroadcaster::TransformGraphBroadcaster(const Config& config)
{
    // load data from config
    for (const auto& marker : config.markers())
        m_markers[marker.ref].push_back(marker);

    for (const auto& entity : config.entities())
    {
        m_entitySensors[entity.name] = entity.sensors;
        m_filters[entity.name].setTimeout(ros::Duration(0.25));
        m_filters[entity.name].setAlpha(0.1);
    }

    m_worldSensors = config.worldSensors();

    m_publishWorldSensors  = config.options().publishWorldSensors;
    m_publishEntitySensors = config.options().publishEntitySensors;
    m_publishMarkers       = config.options().publishMarkers;
}

void TransformGraphBroadcaster::broadcast(const TransformGraph& graph)
{
    auto entityNames = graph.entities();
    for (const auto& entityName : entityNames)
    {
        Pose pose;
        try
        {
            // this method throws if a lookup is not possible
            pose = graph.lookupPose(entityName);
            m_filters[entityName].addPose(pose);
            pose = m_filters[entityName].pose();

            // broadcast the entity's pose in world frame
            if (entityName != "world")
                broadcast("world", entityName, pose);

            if (m_publishMarkers)
            {
                // show the markers attached to that entity
                for (const auto& marker : m_markers[entityName])
                    broadcast(entityName, "Marker " + std::to_string(marker.id), marker.transf);
            }

            if (m_publishEntitySensors)
            {
                // show the sensors attached to that entity
                for (const auto& sensor : m_entitySensors[entityName])
                    broadcast(entityName, entityName + "-" + sensor.name, sensor.transf);
            }
        }
        catch (const std::string&)
        {
            // no lookup possible
        }
    }

    if (m_publishWorldSensors)
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
