#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include "config.h"
#include "filters.h"
#include "transformgraph.h"

class TransformGraphBroadcaster
{
public:
    TransformGraphBroadcaster(const Config& config);

    void broadcast(const TransformGraph& graph);

protected:
    void broadcast(const std::string& frame, const std::string& child, const tf2::Transform transf);
    void broadcast(const std::string& frame, const std::string& child, const Pose pose);

private:
    tf2_ros::TransformBroadcaster m_tfbc;

    std::map<std::string, ExplonentialMovingAverageFilter> m_filters;
    std::map<std::string, Entity> m_entities;

    bool m_publishMarkers       = true;
    bool m_publishEntitySensors = true;
    bool m_publishWorldSensors  = true;
};
