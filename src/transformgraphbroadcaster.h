#pragma once

#include <ros/publisher.h>
#include <tf2_ros/transform_broadcaster.h>

#include "config.h"
#include "filters.h"
#include "transformgraph.h"

/**
 * @brief The TransformGraphBroadcaster class
 * Interface between the transform graph and ROS
 */
class TransformGraphBroadcaster
{
public:
    TransformGraphBroadcaster(const Config& config);

    /**
     * @brief broadcast
     * @param graph: The graph to publish
     */
    void broadcast(const TransformGraph& graph);

protected:
    void broadcast(const std::string& frame, const std::string& child, const tf2::Transform transf);
    void broadcast(const std::string& frame, const std::string& child, const Pose pose);
    void broadcast(const std::string& entity, const Pose pose);

private:
    tf2_ros::TransformBroadcaster m_tfbc;
    ros::Publisher m_dotGraphPublisher;
    std::map<std::string, ros::Publisher> m_publishers;
    ros::NodeHandle m_node;

    std::map<std::string, ExplonentialMovingAverageFilter> m_filters;
    std::map<std::string, Entity> m_entities;

    bool m_publishMarkers       = true;
    bool m_publishEntitySensors = true;
    bool m_publishWorldSensors  = true;
    bool m_publishDotGraph      = true;
    bool m_publishPoseTopics    = true;
};
