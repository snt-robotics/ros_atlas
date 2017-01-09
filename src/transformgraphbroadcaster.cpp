#include "transformgraphbroadcaster.h"

#include <geometry_msgs/TransformStamped.h>

TransformGraphBroadcaster::TransformGraphBroadcaster()
{
}

void TransformGraphBroadcaster::broadcast(const TransformGraph& graph)
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
        }
        catch (const std::string&)
        {
            // no lookup possible
        }
    }
}
