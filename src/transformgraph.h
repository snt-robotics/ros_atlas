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

struct VertexInfo
{
    std::string name;

    // world pose
    tf2::Vector3 pos;
    tf2::Quaternion rot = tf2::Quaternion::getIdentity();

    //
    WeightedMean filter;
    bool evaluated = false;
    int level      = 0;
};

class TransformGraph
{

    // boost graph glue
    struct edgeInfo_t
    {
        typedef boost::edge_property_tag kind;
    };

    struct vertexInfo_t
    {
        typedef boost::vertex_property_tag kind;
    };

    using vertex_descriptor = boost::adjacency_list_traits<boost::listS, boost::listS, boost::directedS>::vertex_descriptor;
    using VertexProp        = boost::property<boost::vertex_index_t, int, boost::property<boost::vertex_distance_t, int, boost::property<boost::vertex_predecessor_t, vertex_descriptor, boost::property<vertexInfo_t, VertexInfo> > > >;
    using EdgePropInfo      = boost::property<edgeInfo_t, EdgeInfo>;
    using EdgeProp          = boost::property<boost::edge_weight_t, double, EdgePropInfo>;
    using Graph             = boost::adjacency_list<boost::multisetS, boost::listS, boost::bidirectionalS, VertexProp, EdgeProp, boost::multisetS>;
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
            const auto& infoMap = boost::get(edgeInfo_t(), graph);
            return infoMap[edge].sensorData.key == key;
        }
    };

public:
    /**
     * @brief TransformGraph creates an empty graph only containing a "world" entity
     */
    TransformGraph();

    /**
     * @brief addEntity creates a named vertex in the graph
     * @param name
     */
    void addEntity(const std::string& name);

    /**
     * @brief updateSensorData
     * @param from: The entity name the sensor information in coming from
     * @param to: The entity name the sensor information targets
     * @param sensorData: The sensor information itself
     */
    void updateSensorData(const std::string& from, const std::string& to, const SensorData& sensorData);
    void removeAllEdges(const std::string& from, const std::string& to);

    /**
     * @brief removeEdgeByKey remoes the edges assigned to a specific sensor information
     * @param key: The key of a specific sensor information
     */
    void removeEdgeByKey(const MeasurementKey& key);

    std::vector<tf2::Transform> edgeTransforms(const std::string& from, const std::string& to);
    std::vector<std::string> lookupPath(const std::string& from, const std::string& to);
    tf2::Transform lookupTransform(const std::string& from, const std::string& to);

    bool canTransform(const std::string& from, const std::string& to);

    /**
     * @brief numberOfEdges
     * @return The number of edges in the graph
     */
    std::size_t numberOfEdges() const;

    /**
     * @brief eval calculates the pose of every entity in the graph
     */
    void eval();

    /**
     * @brief save saves the graph to a given dot file
     * @param filename
     */
    void save(const std::string& filename);

private:
    // the graph container containing all our sensor information
    Graph m_graph;

    // keep track of the vertices by name for easy access
    std::map<std::string, Vertex> m_labeledVertex;

    // used to assign unique IDs to vertices
    int m_count = 0;
};
