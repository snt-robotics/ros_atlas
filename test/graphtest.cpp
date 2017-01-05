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
    graph.updateSensorData("A", "B", { { "A", "testSensor", 0 } });
    ASSERT_EQ(2, graph.numberOfEdges());
    graph.updateSensorData("D", "E", { { "D", "testSensor", 0 } });
    ASSERT_EQ(4, graph.numberOfEdges());
    graph.updateSensorData("D", "E", { { "D", "testSensor", 1 } });
    ASSERT_EQ(6, graph.numberOfEdges());
    //graph.updateSensorData("A", "B", SensorData());
    //graph.updateSensorData("D", "E", SensorData());

    ASSERT_TRUE(graph.canTransform("A", "B"));
    ASSERT_TRUE(graph.canTransform("D", "E"));
    ASSERT_FALSE(graph.canTransform("A", "C"));

    graph.removeEdgeByKey({ "A", "testSensor", 0 });

    ASSERT_FALSE(graph.canTransform("A", "B"));
    ASSERT_EQ(4, graph.numberOfEdges());
}

TEST(Graphs, printTest)
{
    TransformGraph graph;
    graph.addEntity("A");
    graph.addEntity("B");
    graph.addEntity("C");

    graph.updateSensorData("world", "A", { { "world", "optitrack", -1 } });
    graph.updateSensorData("A", "world", { { "A", "cam1", 0 } });
    graph.updateSensorData("A", "world", { { "A", "cam1", 1 } });

    graph.updateSensorData("A", "B", { { "A", "cam0", 2 } });
    graph.updateSensorData("B", "C", { { "B", "cam0", 3 } });

    graph.save("/home/paul/graph1.dot");

    // always successful, requires visual inspection
    ASSERT_TRUE(true);
}

TEST(Graphs, cycleTest)
{
    TransformGraph graph;
    graph.addEntity("A");
    graph.addEntity("B");
    graph.addEntity("C");

    graph.updateSensorData("world", "A", { { "world", "optitrack", -1 } });
    graph.updateSensorData("world", "B", { { "world", "optitrack", -2 } });

    graph.updateSensorData("A", "B", { { "A", "cam0", 0 } });
    graph.updateSensorData("B", "A", { { "B", "cam0", 1 } });

    graph.updateSensorData("A", "C", { { "A", "cam0", 2 } });
    graph.updateSensorData("B", "C", { { "B", "cam0", 2 } });

    graph.save("/home/paul/graphcycle.dot");

    graph.eval();
}

TEST(Graphs, cycleTestEval)
{
    TransformGraph graph;
    graph.addEntity("A");
    graph.addEntity("B");
    graph.addEntity("C");

    graph.updateSensorData("world", "A", { { "world", "optitrack", -1 }, { 1, 1, 0 } });
    graph.updateSensorData("world", "B", { { "world", "optitrack", -2 }, { 1, -1, 0 } });

    // these should have no effect as they are edges on the same level (i.e. same distance to the world)
    graph.updateSensorData("A", "B", { { "A", "cam0", 0 }, { 0, -100, 0 } });
    graph.updateSensorData("B", "A", { { "B", "cam0", 1 }, { 0, -100, 0 } });

    graph.updateSensorData("A", "C", { { "A", "cam0", 2 }, { 1, 1, 0 } });
    graph.updateSensorData("B", "C", { { "B", "cam0", 2 }, { 1, -1, 0 } });

    graph.eval();
    graph.save("/home/paul/graphcycleEval.dot");
}
