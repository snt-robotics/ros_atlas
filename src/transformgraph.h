#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/labeled_graph.hpp>
#include <ros/time.h>
#include <tf2/LinearMath/Transform.h>

#include "sensorlistener.h"

/**
 * @brief The EdgeInfo struct
 * Contains the information needed to travel from A to B
 * In other words: It contains (one or multiple) sensor data
 * of a specific entity
 */
struct EdgeInfo
{
    EdgeInfo(double weight, const SensorDataList& sensorDataList)
        : weight(weight)
    {
        for (const auto& sensorData : sensorDataList)
            sensorDataMap[sensorData.id] = sensorData;
    }

    EdgeInfo inverse() const
    {
        auto copy = *this;
        for (auto&& keyval : copy.sensorDataMap)
            keyval.second.transf = keyval.second.transf.inverse();

        return copy;
    }

    EdgeInfo() {}

    double weight = 1.0;
    std::map<SensorId, SensorData> sensorDataMap;
};

struct NodeInfo
{
    std::string name;

    // world pose
    tf2::Vector3 pos;
    tf2::Quaternion rot;
};

class TransformGraph
{
    // boost graph glue
    struct info_t
    {
        typedef boost::edge_property_tag kind;
    };

    struct pose_t
    {
        typedef boost::edge_property_tag kind;
    };

    using NodeProp       = boost::property<pose_t, NodeInfo>;
    using EdgePropInfo   = boost::property<info_t, EdgeInfo>;
    using EdgeProp       = boost::property<boost::edge_weight_t, double, EdgePropInfo>;
    using Graph          = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, NodeInfo, EdgeProp>;
    using LabeledGraph   = boost::labeled_graph<Graph, std::string>;
    using Vertex         = boost::graph_traits<LabeledGraph>::vertex_descriptor;
    using Edge           = boost::graph_traits<LabeledGraph>::edge_descriptor;
    using VertexIterator = boost::graph_traits<LabeledGraph>::vertex_iterator;

public:
    TransformGraph();

    void addVertex(const std::string& name);
    void updateEdge(const std::string& from, const std::string& to, const EdgeInfo& info);
    void addEdge(const std::string& from, const std::string& to, const EdgeInfo& info);
    void removeEdge(const std::string& from, const std::string& to);
    EdgeInfo edgeInfo(const std::string& from, const std::string& to);
    bool hasEdge(const std::string& from, const std::string& to);
    std::vector<tf2::Transform> edgeTransforms(const std::string& from, const std::string& to);

    std::vector<std::string> lookupPath(const std::string& from, const std::string& to);
    tf2::Transform lookupTransform(const std::string& from, const std::string& to);

private:
    LabeledGraph m_graph;
};
