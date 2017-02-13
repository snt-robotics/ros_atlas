#include "transformgraph.h"

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graphviz.hpp>

TransformGraph::TransformGraph(double decayDuration)
    : m_decayDuration(decayDuration)
{
    // world is a special entity
    addEntity("world");

    // world is by definition always evaluated
    auto vertexInfo = boost::get(vertexInfo_t(), m_graph);

    vertexInfo[m_labeledVertex["world"]].evaluated = true;
}

TransformGraph::TransformGraph(const Config& config)
    : TransformGraph(config.options().decayDuration)
{
    for (const auto& entity : config.entities())
        addEntity(entity.name);
}

void TransformGraph::addEntity(const std::string& name)
{
    if (hasEntity(name))
        return;

    // adding entities is like adding vertices to the graph
    auto vertex           = boost::add_vertex(m_graph);
    m_labeledVertex[name] = vertex;

    // set the index of the new vertex
    auto indices    = boost::get(boost::vertex_index, m_graph);
    indices[vertex] = m_count++;

    // set the name of the new vertex
    auto vertexInfo         = boost::get(vertexInfo_t(), m_graph);
    vertexInfo[vertex].name = name;
}

bool TransformGraph::hasEntity(const std::string& name) const
{
    auto itr = m_labeledVertex.find(name);
    return itr != m_labeledVertex.end();
}

std::vector<std::string> TransformGraph::entities() const
{
    std::vector<std::string> out;

    for (const auto& keyval : m_labeledVertex)
        out.push_back(keyval.first);

    return out;
}

void TransformGraph::updateSensorData(const Measurement& measurement)
{

    // remove edges if they exist
    removeEdgesByKey(measurement.key);

    // edges do not exist, add them
    auto info = EdgeInfo(measurement);

    if (hasEntity(measurement.key.from) && hasEntity(measurement.key.to))
    {
        boost::add_edge(m_labeledVertex[measurement.key.from], m_labeledVertex[measurement.key.to], { 1.0, info }, m_graph);
        boost::add_edge(m_labeledVertex[measurement.key.to], m_labeledVertex[measurement.key.from], { 1.0, info.inverse() }, m_graph);
    }
    else
    {
        // missing entity
        ROS_WARN("Graph: Missing entity '%s' or '%s'", measurement.key.from.c_str(), measurement.key.to.c_str());
    }
}

void TransformGraph::update(const SensorListener& listener)
{
    auto measurements = listener.filteredSensorData();

    for (const auto& measurement : measurements)
        updateSensorData(measurement);

    removeEdgesOlderThan(m_decayDuration);

    clearEvalFlag();
    eval();
}

void TransformGraph::removeAllEdges(const std::string& entity)
{
    boost::clear_vertex(m_labeledVertex[entity], m_graph);
}

void TransformGraph::removeEdgesByKey(const Measurement::Key& key)
{
    RemovePredicateKey pred(key, m_graph);
    boost::remove_edge_if(pred, m_graph);
}

void TransformGraph::removeEdgesOlderThan(ros::Duration duration)
{
    RemovePredicateDuration pred(duration, ros::Time::now(), m_graph);
    boost::remove_edge_if(pred, m_graph);
}

Pose TransformGraph::lookupPose(const std::string& entityName) const
{
    const auto vertexInfo = boost::get(vertexInfo_t(), m_graph);

    auto itr = m_labeledVertex.find(entityName);
    if (itr != m_labeledVertex.end() && vertexInfo[itr->second].evaluated)
    {
        return vertexInfo[itr->second].pose;
    }

    throw("\"" + entityName + "\" is not connected to \"world\"");
}

std::vector<std::string> TransformGraph::lookupPath(const std::string& from, const std::string& to)
{

    auto d    = boost::get(boost::vertex_distance, m_graph); // distances
    auto p    = boost::get(boost::vertex_predecessor, m_graph); // predecessors
    auto info = boost::get(vertexInfo_t(), m_graph); // vertex info

    auto start = m_labeledVertex[from];
    auto goal  = m_labeledVertex[to];

    boost::dijkstra_shortest_paths(m_graph, start, boost::predecessor_map(p).distance_map(d));

    // print graph
    //    VertexIterator vi, vend;
    //    for (boost::tie(vi, vend) = boost::vertices(m_graph); vi != vend; ++vi)
    //    {
    //        std::cout << "distance(" << *vi << ") = " << d[(*vi)] << ", ";
    //        std::cout << "parent(" << *vi << ") = " << p[*vi] << std::endl;
    //    }

    // travel from goal to start
    std::vector<Edge> path;
    std::vector<std::string> verticesInPath;
    Vertex v = goal; // We want to start at the destination and work our way back to the source
    for (Vertex u = p[v]; // Start by setting 'u' to the destintaion node's predecessor
         u != v; // Keep tracking the path until we get to the source
         v = u, u = p[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
    {
        std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(u, v, m_graph);

        if (!edgePair.second)
            continue;

        Graph::edge_descriptor edge = edgePair.first;

        path.push_back(edge);
    }

    //    std::cout << "Shortest path from " << from << " to " << to << ":" << std::endl;
    for (auto itr = path.rbegin(); itr != path.rend(); ++itr)
    {
        auto sourceVertex = boost::source(*itr, m_graph);
        auto targetVertex = boost::target(*itr, m_graph);

        if (sourceVertex == Graph::null_vertex() || targetVertex == Graph::null_vertex())
            continue;

        const auto& sourceName = info[sourceVertex].name;
        const auto& targetName = info[targetVertex].name;
        //        std::cout << sourceName << " -> " << targetName << " = " << boost::get(boost::edge_weight, m_graph, *itr) << std::endl;

        if (itr == path.rbegin())
            verticesInPath.push_back(sourceName);

        verticesInPath.push_back(targetName);
    }
    //    std::cout << std::endl;
    //    std::cout << "Distance: " << d[goal] << std::endl;

    return verticesInPath;
}

tf2::Transform TransformGraph::lookupTransform(const std::string& from, const std::string& to)
{
    auto path = lookupPath(from, to);

    if (path.empty())
        return tf2::Transform(); //failed to lookup transform

    return tf2::Transform();
}

bool TransformGraph::canTransform(const std::string& from, const std::string& to)
{
    return !lookupPath(from, to).empty();
}

std::size_t TransformGraph::numberOfEdges() const
{
    return boost::num_edges(m_graph);
}

void TransformGraph::eval()
{
    class VertexVisitor : public boost::default_bfs_visitor
    {
    public:
        VertexVisitor(std::vector<Vertex>& vertices, std::map<Vertex, int>& levels)
            : vertices(vertices)
            , levels(levels)
        {
        }

        void discover_vertex(Vertex u, const Graph& graph)
        {
            // don't add the world, as its pose is already known
            const auto& info = boost::get(vertexInfo_t(), graph); // vertex names
            if (info[u].name != "world")
                vertices.push_back(u);
        }

        void tree_edge(Edge e, const Graph&)
        {
            auto source = e.m_source;
            auto target = e.m_target;

            levels[target] = levels[source] + 1;
        }

    private:
        std::vector<Vertex>& vertices;
        std::map<Vertex, int>& levels;
    };

    std::vector<Vertex> vertices;
    std::map<Vertex, int> levels;
    auto vertexVisitor = VertexVisitor(vertices, levels);

    // evaluate vertices on the "same level" first
    boost::breadth_first_search(m_graph, m_labeledVertex["world"], boost::visitor(vertexVisitor));

    auto vInfo = boost::get(vertexInfo_t(), m_graph); // vertex info
    auto eInfo = boost::get(edgeInfo_t(), m_graph); // edge info

    // move the levels information to the vertex properties
    // for easy access
    for (const auto& currentVertex : vertices)
    {
        vInfo[currentVertex].level = levels[currentVertex];
    }

    // evaluate the vertices on the stack
    for (const auto& currentVertex : vertices)
    {
        auto itrs = boost::in_edges(currentVertex, m_graph);

        // find smallest sigma (i.e. the "best" sensor)
        // used to calculate the weight
        auto minItr = std::min_element(itrs.first, itrs.second, [&eInfo](Edge a, Edge b) {
            return eInfo[a].sensorData.sigma < eInfo[b].sensorData.sigma;
        });
        const double minSigma = eInfo[*minItr].sensorData.sigma;

        // evaluate edges
        for (auto edge : boost::make_iterator_range(itrs.first, itrs.second))
        {
            // get the source vertex of that edge
            const auto sourceVertex = edge.m_source;

            // if the source hasn't been evaluated yet, we just skip it
            // as it is of no value to us
            // The same applies to vertices that have the same distance to the world
            if (!vInfo[sourceVertex].evaluated || vInfo[sourceVertex].level >= vInfo[currentVertex].level)
                continue;

            // the source has been evaluated and as such we can use it
            // for the pose calculation
            // The edges contain the transformation
            // The vertices contain the pose
            const auto vertextransform = tf2::Transform{ vInfo[sourceVertex].pose.rot, vInfo[sourceVertex].pose.pos };
            const auto edgetransform   = eInfo[edge].sensorData.transform;

            const auto result = vertextransform * edgetransform;

            // the standard deviation
            const auto sigma = eInfo[edge].sensorData.sigma;

            // the weight. Lower sigmas are weighted higher.
            const auto weight = minSigma / sigma;

            // filter
            vInfo[currentVertex].filter.addVec3(result.getOrigin(), weight);
            vInfo[currentVertex].filter.addQuat(result.getRotation(), weight);
        }

        // get the results from the filter
        vInfo[currentVertex].pose.pos = vInfo[currentVertex].filter.weightedMeanVec3();
        vInfo[currentVertex].pose.rot = vInfo[currentVertex].filter.weightedMeanQuat();

        vInfo[currentVertex].evaluated = true;
        vInfo[currentVertex].filter.reset();
    }
}

void TransformGraph::save(const std::string& filename)
{
    std::ofstream file;
    file.open(filename, std::ofstream::out | std::ofstream::trunc);
    file << toDot();
}

std::string TransformGraph::toDot() const
{
    std::stringstream ss;

    auto vertexInfo = boost::get(vertexInfo_t(), m_graph);
    auto edgeInfo   = boost::get(edgeInfo_t(), m_graph);

    boost::write_graphviz(ss, m_graph, boost::make_label_writer(vertexInfo), boost::make_label_writer(edgeInfo));

    return ss.str();
}

void TransformGraph::clearEvalFlag()
{
    auto vInfo = boost::get(vertexInfo_t(), m_graph);

    for (const auto& keyval : m_labeledVertex)
        if (keyval.first != "world")
            vInfo[keyval.second].evaluated = false;
}

////////////////////////////////////////////////////
// print helpers
////////////////////////////////////////////////////

// The following information is displayed on edges
std::ostream& operator<<(std::ostream& os, const TransformGraph::EdgeInfo& info)
{
    return os << "source: " << info.sensorData.key.from << '\n'
              << "sensor: " << info.sensorData.key.sensor << '\n'
              << "marker: " << info.sensorData.key.marker;
}

// The following information is displayed on vertices
std::ostream& operator<<(std::ostream& os, const TransformGraph::VertexInfo& info)
{
    return os << "Name: " << info.name << '\n'
              << "pos: " << info.pose.pos.x() << "/" << info.pose.pos.y() << "/" << info.pose.pos.z() << '\n'
              << "quat: " << info.pose.rot.x() << "/" << info.pose.rot.y() << "/" << info.pose.rot.z() << "/" << info.pose.rot.w() << '\n'
              << "lvl: " << info.level;
}
