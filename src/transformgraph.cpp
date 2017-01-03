#include "transformgraph.h"

#include <boost/graph/dijkstra_shortest_paths.hpp>

TransformGraph::TransformGraph()
{
}

void TransformGraph::addVertex(const std::string& name)
{
    boost::add_vertex(name, m_graph);
    m_graph[name].name = name;
}

void TransformGraph::updateEdge(const std::string& from, const std::string& to, const EdgeInfo& info)
{
    auto edge = boost::edge(boost::vertex_by_label(from, m_graph), boost::vertex_by_label(to, m_graph), m_graph);

    if (edge.second)
        removeEdge(from, to);

    addEdge(from, to, info);
}

void TransformGraph::addEdge(const std::string& from, const std::string& to, const EdgeInfo& info)
{
    boost::add_edge_by_label(from, to, { info.weight, info }, m_graph); // forth
    boost::add_edge_by_label(to, from, { info.weight, info.inverse() }, m_graph); // and back
}

void TransformGraph::removeEdge(const std::string& from, const std::string& to)
{
    boost::remove_edge_by_label(from, to, m_graph);
}

EdgeInfo TransformGraph::edgeInfo(const std::string& from, const std::string& to)
{
    auto edge = boost::edge(boost::vertex_by_label(from, m_graph), boost::vertex_by_label(to, m_graph), m_graph);
    return boost::get(info_t(), m_graph, edge.first);
}

bool TransformGraph::hasEdge(const std::string& from, const std::string& to)
{
    auto edge = boost::edge(boost::vertex_by_label(from, m_graph), boost::vertex_by_label(to, m_graph), m_graph);
    return edge.second;
}

std::vector<tf2::Transform> TransformGraph::edgeTransforms(const std::string& from, const std::string& to)
{
    assert(hasEdge(from, to));
    auto info = edgeInfo(from, to);
    return info.transform;
}

std::vector<std::string> TransformGraph::lookupPath(const std::string& from, const std::string& to)
{
    std::vector<Vertex> p(boost::num_edges(m_graph)); // predecessors
    std::vector<double> d(boost::num_edges(m_graph)); // distances

    auto start = boost::vertex_by_label(from, m_graph);
    auto goal  = boost::vertex_by_label(to, m_graph);
    boost::dijkstra_shortest_paths(m_graph, start, boost::predecessor_map(&p[0]).distance_map(&d[0]));

    // print graph
    VertexIterator vi, vend;
    for (boost::tie(vi, vend) = boost::vertices(m_graph); vi != vend; ++vi)
    {
        std::cout << "distance(" << *vi << ") = " << d[*vi] << ", ";
        std::cout << "parent(" << *vi << ") = " << p[*vi] << std::endl;
    }

    // travel from goal to start
    std::vector<Edge> path;
    std::vector<std::string> verticesInPath;
    Vertex v = goal; // We want to start at the destination and work our way back to the source
    for (Vertex u = p[v]; // Start by setting 'u' to the destintaion node's predecessor
         u != v; // Keep tracking the path until we get to the source
         v = u, u = p[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
    {
        std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(u, v, m_graph);
        Graph::edge_descriptor edge = edgePair.first;

        path.push_back(edge);
    }

    std::cout << "Shortest path from " << from << " to " << to << ":" << std::endl;
    for (auto itr = path.rbegin(); itr != path.rend(); ++itr)
    {

        const auto& sourceName = m_graph.graph()[boost::source(*itr, m_graph)].name;
        const auto& targetName = m_graph.graph()[boost::target(*itr, m_graph)].name;
        std::cout << sourceName << " -> " << targetName << " = " << boost::get(boost::edge_weight, m_graph, *itr) << std::endl;

        if (itr == path.rbegin())
            verticesInPath.push_back(sourceName);

        verticesInPath.push_back(targetName);
    }
    std::cout << std::endl;
    std::cout << "Distance: " << d[goal] << std::endl;

    return verticesInPath;
}

tf2::Transform TransformGraph::lookupTransform(const std::string& from, const std::string& to)
{
    auto path = lookupPath(from, to);

    if (path.empty())
        return tf2::Transform(); //failed to lookup transform
}
