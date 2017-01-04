#pragma once

#include "sensorlistener.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/labeled_graph.hpp>
#include <ros/time.h>
#include <tf2/LinearMath/Transform.h>

/**
 * @brief The EdgeInfo struct
 * Contains the information needed to travel from A to B
 * In other words: It contains (one or multiple) sensor data
 * of a specific entity
 */
struct EdgeInfo
{
    EdgeInfo(const SensorData& sensorData)
        : sensorData(sensorData)
    {
    }

    EdgeInfo inverse() const
    {
        auto copy = *this;

        copy.sensorData.transform = copy.sensorData.transform.inverse();

        return copy;
    }

    EdgeInfo() {}
    SensorData sensorData;
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
    using Graph          = boost::adjacency_list<boost::multisetS, boost::listS, boost::bidirectionalS, NodeInfo, EdgeProp, boost::multisetS>;
    using LabeledGraph   = boost::labeled_graph<Graph, std::string>;
    using Vertex         = boost::graph_traits<LabeledGraph>::vertex_descriptor;
    using Edge           = boost::graph_traits<LabeledGraph>::edge_descriptor;
    using VertexIterator = boost::graph_traits<LabeledGraph>::vertex_iterator;

    struct RemovePredicate
    {
        RemovePredicate(const MeasurementKey& key, const Graph& graph)
            : key(key)
            , graph(graph)
        {
        }

        const MeasurementKey& key;
        const Graph& graph;
        mutable bool stop = false;

        bool operator()(const Edge& edge) const
        {
            if (stop)
                return false;

            const auto& infoMap = boost::get(info_t(), graph);

            stop = infoMap[edge].sensorData.key == key;

            if (stop)
            {
                std::cout << "remove edge " << key.entity << key.sensor << " edge " << edge << std::endl;
            }

            return stop;
        }
    };

public:
    TransformGraph();

    void addEntity(const std::string& name);
    void updateSensorData(const std::string& from, const std::string& to, const SensorData& sensorData);
    void removeAllEdges(const std::string& from, const std::string& to);
    void removeEdgeByKey(const MeasurementKey& key);

    std::vector<tf2::Transform> edgeTransforms(const std::string& from, const std::string& to);
    std::vector<std::string> lookupPath(const std::string& from, const std::string& to);
    tf2::Transform lookupTransform(const std::string& from, const std::string& to);

    bool canTransform(const std::string& from, const std::string& to);

    std::size_t numberOfEdges() const;

private:
    LabeledGraph m_graph;
};
