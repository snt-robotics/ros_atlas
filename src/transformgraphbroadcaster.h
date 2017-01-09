#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include "config.h"
#include "transformgraph.h"

class TransformGraphBroadcaster
{
public:
    TransformGraphBroadcaster(const Config& config);

    void broadcast(const TransformGraph& graph, bool publishMarkers = false, bool publishEntitySensors = false, bool publishWorldSensors = false);

private:
    tf2_ros::TransformBroadcaster m_tfbc;

    std::map<std::string, std::vector<Marker> > m_markers;
    std::map<std::string, std::vector<Sensor> > m_entitySensors;
    std::vector<WorldSensor> m_worldSensors;
};
