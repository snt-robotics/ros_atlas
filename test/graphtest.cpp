#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>

#include "../src/transformgraph.h"
#include "helpers.h"

TEST(Graphs, transformgraph)
{
    TransformGraph graph;
    graph.addEntity("A");
    graph.addEntity("B");
    graph.addEntity("C");
    graph.addEntity("D");
    graph.addEntity("E");
    graph.updateSensorData("A", "B", { { "testEntity1", "testSensor", 0 }, { 0, 0, 0 }, { 0, 0, 0, 1 } });
    graph.updateSensorData("D", "E", { { "testEntity2", "testSensor", 0 }, { 0, 0, 0 }, { 0, 0, 0, 1 } });
    //graph.updateSensorData("A", "B", SensorData());
    //graph.updateSensorData("D", "E", SensorData());

    ASSERT_TRUE(graph.canTransform("A", "B"));
    ASSERT_FALSE(graph.canTransform("A", "C"));
    ASSERT_EQ(4, graph.numberOfEdges());

    graph.removeEdgeByKey({ "testEntity1", "testSensor", 0 });

    ASSERT_FALSE(graph.canTransform("A", "B"));
    ASSERT_EQ(2, graph.numberOfEdges());

    //graph.updateSensorData("A", "B", SensorData());
    //ASSERT_TRUE(graph.canTransform("A", "B"));

    //    graph.updateSensorData("A", "B", { { "testEntity", "testSensor", 0 }, { 10, 0, 0 }, { 0, 0, 0, 1 } });
    //    graph.updateSensorData("B", "C", { { "testEntity", "testSensor", 0 }, { 0, 10, 0 }, { 0, 0, 0, 1 } });

    auto expected = tf2::Transform{ { 0, 0, 0, 1 }, { 10, 0, 0 } };
    //ASSERT_EQ(expected, graph.edgeInfo("A", "B").transform);

    // there is a path from A to C
    //    ASSERT_TRUE(pathEq({ "A", "B", "C" }, graph.lookupPath("A", "C")));

    //    // there is no path from A to E
    //    ASSERT_TRUE(graph.lookupPath("A", "E").empty());
}

//TEST(Graphs, cycleTest)
//{
//    TransformGraph graph;
//    graph.addVertex("world");
//    graph.addVertex("A");
//    graph.addVertex("B");
//    graph.addVertex("C");

//    graph.addEdge("world", "A", EdgeInfo());
//    graph.addEdge("A", "B", EdgeInfo());
//    graph.addEdge("A", "C", EdgeInfo());
//    graph.addEdge("B", "C", EdgeInfo());

//    // there is a path from A to C
//    ASSERT_TRUE(pathEq({ "world", "A", "C" }, graph.lookupPath("world", "C")));
//}
