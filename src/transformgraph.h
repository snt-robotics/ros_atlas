#pragma once

#include "config.h"
#include "filters.h"
#include "helpers.h"
#include "sensorlistener.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <ros/time.h>
#include <tf2/LinearMath/Transform.h>

class TransformGraph
{
    /**
   * @brief The EdgeInfo struct
   * Contains the information needed to travel from A to B
   * In other words: It contains (one or multiple) sensor data
   * of a specific entity
   */
    struct EdgeInfo
    {
        EdgeInfo(const Measurement& sensorData)
            : sensorData(sensorData)
        {
        }

        EdgeInfo() {}

        friend std::ostream& operator<<(std::ostream& os, const EdgeInfo& info);

        EdgeInfo inverse() const
        {
            auto copy = *this;

            copy.sensorData.transform = copy.sensorData.transform.inverse();

            return copy;
        }

        Measurement sensorData;
    };

    /**
     * @brief The VertexInfo struct
     * Contains all the data associated with a given vertex
     */
    struct VertexInfo
    {
        std::string name;

        // the pose in the world frame
        Pose pose;

        //
        WeightedMean filter; // filter used to fuse the sensor data
        bool evaluated = false;
        int level      = 0;

        friend std::ostream& operator<<(std::ostream& os, const VertexInfo& info);
    };

    friend std::ostream& operator<<(std::ostream& os, const TransformGraph::EdgeInfo& info);
    friend std::ostream& operator<<(std::ostream& os, const TransformGraph::VertexInfo& info);

    // boost graph property glue
    struct edgeInfo_t
    {
        typedef boost::edge_property_tag kind;
    };

    struct vertexInfo_t
    {
        typedef boost::vertex_property_tag kind;
    };

    // boost graph shortcuts
    using vertex_descriptor = boost::adjacency_list_traits<boost::listS, boost::listS, boost::directedS>::vertex_descriptor;
    using VertexProp        = boost::property<boost::vertex_index_t, int, boost::property<boost::vertex_distance_t, int, boost::property<boost::vertex_predecessor_t, vertex_descriptor, boost::property<vertexInfo_t, VertexInfo> > > >;
    using EdgePropInfo      = boost::property<edgeInfo_t, EdgeInfo>;
    using EdgeProp          = boost::property<boost::edge_weight_t, double, EdgePropInfo>;
    using Graph             = boost::adjacency_list<boost::multisetS, boost::listS, boost::bidirectionalS, VertexProp, EdgeProp, boost::multisetS>;
    using Vertex            = boost::graph_traits<Graph>::vertex_descriptor;
    using Edge              = boost::graph_traits<Graph>::edge_descriptor;
    using InEdgeIterator    = boost::graph_traits<Graph>::in_edge_iterator;
    using VertexIterator    = boost::graph_traits<Graph>::vertex_iterator;

    /**
     * @brief The RemovePredicateKey struct
     * Used to remove specific edges from the graph by key
     */
    struct RemovePredicateKey
    {
        RemovePredicateKey(const Measurement::Key& key, const Graph& graph)
            : key(key)
            , graph(graph)
        {
        }

        const Measurement::Key& key;
        const Graph& graph;

        bool operator()(const Edge& edge) const
        {
            const auto& infoMap = boost::get(edgeInfo_t(), graph);
            return infoMap[edge].sensorData.key == key;
        }
    };

    /**
     * @brief The RemovePredicateDuration struct
     * Used to remove specific edges from the graph by duration
     */
    struct RemovePredicateDuration
    {
        RemovePredicateDuration(ros::Duration duration, ros::Time now, const Graph& graph)
            : duration(duration)
            , now(now)
            , graph(graph)
        {
        }

        ros::Duration duration;
        ros::Time now;
        const Graph& graph;

        bool operator()(const Edge& edge) const
        {
            const auto& infoMap = boost::get(edgeInfo_t(), graph);
            return (now - infoMap[edge].sensorData.stamp) >= duration;
        }
    };

public:
    /**
     * @brief TransformGraph creates an empty graph only containing a "world" entity
     */
    TransformGraph(double decayDuration = 0.25);

    /**
     * @brief TransformGraph creates an empty graph only containing a "world" entity
     */
    TransformGraph(const Config& config);

    /**
     * @brief addEntity creates a named vertex in the graph
     * @param name
     */
    void addEntity(const std::string& name);

    /**
     * @brief hasEntity
     * @param name
     * @return Returns true if the entity exists
     */
    bool hasEntity(const std::string& name) const;

    /**
     * @brief entities returns a list containing all the entities in the graph
     * @return Returns a list of entity names
     */
    std::vector<std::string> entities() const;

    /**
     * @brief updateSensorData
     * @param from: The entity name the sensor information in coming from
     * @param to: The entity name the sensor information targets
     * @param sensorData: The sensor information itself
     */
    void updateSensorData(const Measurement& measurement);

    /**
     * @brief update updates from a sensor listener and removes expired edges, also evaluates the graph
     * @param listener: Used to update the graph
     */
    void update(const SensorListener& listener);

    /**
     * @brief removeAllEdges removes all sensor data assigned to a given entity
     * @param entity
     */
    void removeAllEdges(const std::string& entity);

    /**
     * @brief removeEdgesByKey remoes the edges assigned to a specific sensor information
     * @param key: The key of a specific sensor information
     */
    void removeEdgesByKey(const Measurement::Key& key);

    /**
     * @brief removeEdgesOlderThan breaks edges that are older than a given duration. Use this
     * to get rid of sensor data which are too old to be useful
     * @param duration
     */
    void removeEdgesOlderThan(ros::Duration duration);

    /**
     * @brief lookupPose returns the pose of a given entity. Throws if the lookup is not successful.
     * @param entity
     * @return The pose of the given entity
     */
    Pose lookupPose(const std::string& entityName) const;

    /**
     * @todo Remove?
     */
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

    /**
     * @brief clearEvalFlag
     * Clears the "evaluated" flag of all entites
     */
    void clearEvalFlag();

private:
    // the graph container containing all our sensor information
    Graph m_graph;

    // keep track of the vertices by name for easy access
    std::map<std::string, Vertex> m_labeledVertex;

    // used to assign unique IDs to vertices
    int m_count = 0;

    // decay duration i.e. the time after which edges with no activity are removed
    ros::Duration m_decayDuration = ros::Duration(0.25);
};
