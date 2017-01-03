#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>

#include "../src/transformgraph.h"
#include "helpers.h"

TEST(Graphs, transformgraph)
{
    TransformGraph graph;
    graph.addVertex("A");
    graph.addVertex("B");
    graph.addVertex("C");
    graph.addVertex("D");
    graph.addVertex("E");
    graph.addEdge("A", "B", EdgeInfo());
    graph.addEdge("D", "E", EdgeInfo());

    ASSERT_TRUE(graph.hasEdge("A", "B"));
    ASSERT_FALSE(graph.hasEdge("A", "C"));

    graph.removeEdge("A", "B");
    ASSERT_FALSE(graph.hasEdge("A", "B"));

    graph.updateEdge("A", "B", EdgeInfo());
    ASSERT_TRUE(graph.hasEdge("A", "B"));

    graph.updateEdge("A", "B", { 1.0, tf2::Transform{ { 0, 0, 0, 1 }, { 10, 0, 0 } } });
    graph.updateEdge("B", "C", { 1.0, tf2::Transform{ { 0, 0, 0, 1 }, { 0, 10, 0 } } });
    auto expected = tf2::Transform{ { 0, 0, 0, 1 }, { 10, 0, 0 } };
    ASSERT_EQ(expected, graph.edgeInfo("A", "B").transform);

    // there is a path from A to C
    ASSERT_TRUE(pathEq({ "A", "B", "C" }, graph.lookupPath("A", "C")));

    // there is no path from A to E
    ASSERT_TRUE(graph.lookupPath("A", "E").empty());
}

TEST(Graphs, cycleTest)
{
    TransformGraph graph;
    graph.addVertex("world");
    graph.addVertex("A");
    graph.addVertex("B");
    graph.addVertex("C");

    graph.addEdge("world", "A", EdgeInfo());
    graph.addEdge("A", "B", EdgeInfo());
    graph.addEdge("A", "C", EdgeInfo());
    graph.addEdge("B", "C", EdgeInfo());

    // there is a path from A to C
    ASSERT_TRUE(pathEq({ "world", "A", "C" }, graph.lookupPath("world", "C")));
}
