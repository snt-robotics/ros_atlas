#include "transformgraphbroadcaster.h"
#include "std_msgs/String.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

TransformGraphBroadcaster::TransformGraphBroadcaster(const Config& config)
{
    // load entities
    for (const auto& entity : config.entities())
    {
        m_entities[entity.name] = entity;

        // cfg filter
        m_filters[entity.name].setTimeout(ros::Duration(0.25));
        m_filters[entity.name].setAlpha(0.1);
    }

    m_publishWorldSensors  = config.options().publishWorldSensors;
    m_publishEntitySensors = config.options().publishEntitySensors;
    m_publishMarkers       = config.options().publishMarkers;

    // create publisher
    m_dotGraphPublisher = m_node.advertise<std_msgs::String>("transformgraph", 10);

    // create publish as topic publishers
    if (m_publishPoseTopics)
    {
        for (const auto& entity : config.entities())
            m_publishers[entity.name] = m_node.advertise<geometry_msgs::PoseStamped>("atlas/poses/" + entity.name, 10);
    }
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

            // filter the pose
            m_filters[entityName].addPose(pose);
            pose = m_filters[entityName].pose();

            // broadcast the entity's pose in world frame
            if (entityName != "world")
                broadcast("world", entityName, pose);

            if (m_publishMarkers)
            {
                // show the markers attached to that entity
                for (const auto& marker : m_entities[entityName].markers)
                    broadcast(entityName, "Marker " + std::to_string(marker.id), marker.transf);
            }

            if (m_publishEntitySensors)
            {
                // show the sensors attached to that entity
                for (const auto& sensor : m_entities[entityName].sensors)
                    broadcast(entityName, entityName + "-" + sensor.name, sensor.transf);
            }

            if (m_publishPoseTopics)
            {
                broadcast(entityName, pose);
            }
        }
        catch (const std::string&)
        {
            // no lookup possible
        }
    }

    if (m_publishDotGraph)
    {
        std_msgs::String msg;
        msg.data = graph.toDot();
        m_dotGraphPublisher.publish(msg);
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

void TransformGraphBroadcaster::broadcast(const std::string& entity, const Pose pose)
{
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.stamp    = ros::Time::now();
    poseMsg.header.frame_id = "world";

    poseMsg.pose.orientation.x = pose.rot.x();
    poseMsg.pose.orientation.y = pose.rot.y();
    poseMsg.pose.orientation.z = pose.rot.z();
    poseMsg.pose.orientation.w = pose.rot.w();

    poseMsg.pose.position.x = pose.pos.x();
    poseMsg.pose.position.y = pose.pos.y();
    poseMsg.pose.position.z = pose.pos.z();

    m_publishers[entity].publish(poseMsg);
}
