#include <ros/ros.h>

#include "config.h"
#include "sensorlistener.h"
#include "transformgraph.h"
#include "transformgraphbroadcaster.h"

int main(int argc, char** argv)
{
    // init ros
    ros::init(argc, argv, "atlas");

    // get config directory
    ros::NodeHandle n("~");
    auto configFile = n.param<std::string>("config", "");

    assert(!configFile.empty());

    // init atlas
    Config config(configFile);
    SensorListener sensorListener(config);
    TransformGraph graph;
    TransformGraphBroadcaster broadcaster(config);

    // print the config
    config.dump();

    //////////////////////////////////////
    ///      Main Loop
    //////////////////////////////////////
    ros::Rate loopRate(60);
    while (ros::ok())
    {
        graph.update(sensorListener, ros::Duration(100.0 / 1000.0));
        broadcaster.broadcast(graph, true, true);
        sensorListener.clear();

        loopRate.sleep();
    }
}
