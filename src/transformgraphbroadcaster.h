/*
 * ATLAS - Cooperative sensing
 * Copyright (C) 2017  Paul KREMER
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
    void broadcast(const std::string& entity, const Pose pose, int fuseCount);

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
