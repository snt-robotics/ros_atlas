#include "rvizhelpers.h"

RvizHelpers::RvizHelpers(ros::NodeHandle& handle)
{
    rvisPub = handle.advertise<visualization_msgs::Marker>("visualization_marker", 0);
}

void RvizHelpers::publishMarker(int id, const tf::Transform& transf)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id    = "map";
    marker.header.stamp       = ros::Time();
    marker.ns                 = "my_namespace";
    marker.id                 = id;
    marker.type               = visualization_msgs::Marker::CUBE;
    marker.action             = visualization_msgs::Marker::ADD;
    marker.pose.position.x    = transf.getOrigin().x();
    marker.pose.position.y    = transf.getOrigin().y();
    marker.pose.position.z    = transf.getOrigin().z();
    marker.pose.orientation.x = transf.getRotation().x();
    marker.pose.orientation.y = transf.getRotation().y();
    marker.pose.orientation.z = transf.getRotation().z();
    marker.pose.orientation.w = transf.getRotation().w();
    marker.scale.x            = 0.1;
    marker.scale.y            = 0.1;
    marker.scale.z            = 0.1;
    marker.color.a            = 1.0;
    marker.color.r            = 0.0;
    marker.color.g            = 1.0;
    marker.color.b            = 0.0;
    rvisPub.publish(marker);
}
