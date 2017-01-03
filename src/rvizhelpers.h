#pragma once

#include <ros/node_handle.h>
#include <tf/LinearMath/Transform.h>
#include <visualization_msgs/Marker.h>

class RvizHelpers
{
public:
    RvizHelpers(ros::NodeHandle& handle);

    void publishMarker(int id, const tf::Transform& transf);

private:
    ros::Publisher rvisPub;
};
