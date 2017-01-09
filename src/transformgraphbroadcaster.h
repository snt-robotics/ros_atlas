#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include "transformgraph.h"

class TransformGraphBroadcaster
{
public:
    TransformGraphBroadcaster();

    void broadcast(const TransformGraph& graph);

private:
    tf2_ros::TransformBroadcaster m_tfbc;
};
