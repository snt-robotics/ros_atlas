#include <angles/angles.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>

#include <chrono>
#include <thread>

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
    graph.updateSensorData({ { "A", "B", "testSensor", 0 } });
    ASSERT_EQ(2, graph.numberOfEdges());
    graph.updateSensorData({ { "D", "E", "testSensor", 0 } });
    ASSERT_EQ(4, graph.numberOfEdges());
    graph.updateSensorData({ { "D", "E", "testSensor", 1 } });
    ASSERT_EQ(6, graph.numberOfEdges());
    //graph.updateSensorData("A", "B", SensorData());
    //graph.updateSensorData("D", "E", SensorData());

    ASSERT_TRUE(graph.canTransform("A", "B"));
    ASSERT_TRUE(graph.canTransform("D", "E"));
    ASSERT_FALSE(graph.canTransform("A", "C"));

    graph.removeEdgesByKey({ "A", "B", "testSensor", 0 });

    ASSERT_FALSE(graph.canTransform("A", "B"));
    ASSERT_EQ(4, graph.numberOfEdges());
}

TEST(Graphs, printTest)
{
    TransformGraph graph;
    graph.addEntity("A");
    graph.addEntity("B");
    graph.addEntity("C");

    graph.updateSensorData({ { "world", "A", "optitrack", -1 } });
    graph.updateSensorData({ { "A", "world", "cam1", 0 } });
    graph.updateSensorData({ { "A", "world", "cam1", 1 } });

    graph.updateSensorData({ { "A", "B", "cam0", 2 } });
    graph.updateSensorData({ { "B", "C", "cam0", 3 } });

    graph.save("atlas/Testing/graph1.dot");

    // always successful, requires visual inspection
    ASSERT_TRUE(true);
}

TEST(Graphs, cycleTest)
{
    TransformGraph graph;
    graph.addEntity("A");
    graph.addEntity("B");
    graph.addEntity("C");

    graph.updateSensorData({ { "world", "A", "optitrack", -1 } });
    graph.updateSensorData({ { "world", "B", "optitrack", -2 } });

    graph.updateSensorData({ { "A", "B", "cam0", 0 } });
    graph.updateSensorData({ { "B", "A", "cam0", 1 } });

    graph.updateSensorData({ { "A", "C", "cam0", 2 } });
    graph.updateSensorData({ { "B", "C", "cam0", 2 } });

    graph.save("atlas/Testing/graphcycle.dot");

    graph.eval();
}

TEST(Graphs, cycleTestEval)
{
    TransformGraph graph;
    graph.addEntity("A");
    graph.addEntity("B");
    graph.addEntity("C");

    graph.updateSensorData({ { "world", "A", "optitrack", -1 }, { 1, 1, 0 } });
    graph.updateSensorData({ { "world", "B", "optitrack", -2 }, { 1, -1, 0 } });

    // these should have no effect as they are edges on the same level (i.e. same distance to the world)
    graph.updateSensorData({ { "A", "B", "cam0", 0 }, { 0, -100, 0 } });
    graph.updateSensorData({ { "B", "A", "cam0", 1 }, { 0, -100, 0 } });

    graph.updateSensorData({ { "A", "C", "cam0", 2 }, { 1, 1, 0 } });
    graph.updateSensorData({ { "B", "C", "cam0", 2 }, { 1, -1, 0 } });

    graph.eval();
    graph.save("atlas/Testing/graphcycleEval.dot");

    // check poses
    ASSERT_TRUE(poseEq({ { 1, 1, 0 }, { 0, 0, 0, 1 } }, graph.lookupPose("A")));
    ASSERT_TRUE(poseEq({ { 1, -1, 0 }, { 0, 0, 0, 1 } }, graph.lookupPose("B")));
    ASSERT_TRUE(poseEq({ { 2, 0, 0 }, { 0, 0, 0, 1 } }, graph.lookupPose("C")));
}

TEST(Graphs, sigmaTest)
{
    TransformGraph graph;
    graph.addEntity("A");
    graph.addEntity("B");
    graph.addEntity("C");

    // the first sensor is really good compared to the second
    // we expect the result to be almost equal to the position
    // detected by the first sensor.
    graph.updateSensorData({ { "world", "A", "optitrack", -1 }, { 1, 1, 0 }, 0.000001 });
    graph.updateSensorData({ { "world", "B", "optitrack", -2 }, { 1, -1, 0 }, 1.0 });

    graph.eval();

    // check poses
    ASSERT_TRUE(poseEq({ { 1, 1, 0 }, { 0, 0, 0, 1 } }, graph.lookupPose("A")));
}

TEST(Graphs, expire)
{
    TransformGraph graph;
    graph.addEntity("A");
    graph.updateSensorData({ { "world", "A", "optitrack", -1 }, { 1, 1, 0 } });

    ASSERT_TRUE(graph.canTransform("world", "A"));

    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // sleep for 20ms
    graph.removeEdgesOlderThan(ros::Duration(10.0 / 1000.0)); // older than 10ms

    ASSERT_FALSE(graph.canTransform("world", "A"));

    graph.updateSensorData({ { "world", "A", "optitrack", -1 }, { 1, 1, 0 } });
    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // sleep for 20ms
    graph.removeEdgesOlderThan(ros::Duration(30.0 / 1000.0)); // older than 30ms

    ASSERT_TRUE(graph.canTransform("world", "A"));
}

TEST(Graphs, expectThrow)
{
    TransformGraph graph;
    graph.addEntity("A");
    graph.addEntity("B");
    graph.updateSensorData({ { "A", "B", "cam0", 0 }, { 1, 1, 0 } });
    graph.eval();

    EXPECT_ANY_THROW(graph.lookupPose("A"));
    EXPECT_ANY_THROW(graph.lookupPose("B"));

    auto q = tf2::Quaternion({ 0, 1, 0 }, angles::from_degrees(90 + 28)) * tf2::Quaternion({ 0, 0, 1 }, angles::from_degrees(270));
    std::cout << q << std::endl;
}
