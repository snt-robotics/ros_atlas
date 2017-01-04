#pragma once

#include "filters.h"
#include "sensorlistener.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
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

std::ostream& operator<<(std::ostream& os, const EdgeInfo& info);

struct NodeInfo
{
    std::string name;

    // world pose
    tf2::Vector3 pos;
    tf2::Quaternion rot;

    //
    WeightedMean filter;
    bool evaluated = false;
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
        typedef boost::vertex_property_tag kind;
    };

    using vertex_descriptor = boost::adjacency_list_traits<boost::listS, boost::listS, boost::directedS>::vertex_descriptor;
    using NodeProp          = boost::property<boost::vertex_index_t, int, boost::property<boost::vertex_name_t, std::string, boost::property<boost::vertex_distance_t, int, boost::property<boost::vertex_predecessor_t, vertex_descriptor, boost::property<pose_t, NodeInfo> > > > >;
    using EdgePropInfo      = boost::property<info_t, EdgeInfo>;
    using EdgeProp          = boost::property<boost::edge_weight_t, double, EdgePropInfo>;
    using Graph             = boost::adjacency_list<boost::multisetS, boost::listS, boost::bidirectionalS, NodeProp, EdgeProp, boost::multisetS>;
    using Vertex            = boost::graph_traits<Graph>::vertex_descriptor;
    using Edge              = boost::graph_traits<Graph>::edge_descriptor;
    using VertexIterator    = boost::graph_traits<Graph>::vertex_iterator;

    struct RemovePredicate
    {
        RemovePredicate(const MeasurementKey& key, const Graph& graph)
            : key(key)
            , graph(graph)
        {
        }

        const MeasurementKey& key;
        const Graph& graph;

        bool operator()(const Edge& edge) const
        {
            const auto& infoMap = boost::get(info_t(), graph);
            return infoMap[edge].sensorData.key == key;
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

    void eval();

    void save(const std::string& filename);

private:
    Graph m_graph;
    std::map<std::string, Vertex> m_labeledVertex;
    int m_vCount = 0;
};
