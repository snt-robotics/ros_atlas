#include <ros/ros.h>

#include "config.h"
#include "sensorlistener.h"
#include "transformgraph.h"
#include "transformgraphbroadcaster.h"

int main(int argc, char** argv)
{
    const std::vector<std::string> args(argv + 1, argv + argc);

    // make sure we have a config file
    assert(args.size() > 1);

    // init ros
    ros::init(argc, argv, "atlas");

    // init atlas
    Config config(args[1]);
    SensorListener sensorListener(config);
    TransformGraph graph;
    TransformGraphBroadcaster broadcaster;

    //////////////////////////////////////
    ///      Main Loop
    //////////////////////////////////////
    ros::Rate loopRate(60);
    while (ros::ok())
    {
        graph.update(sensorListener, ros::Duration(100.0 / 1000.0));
        broadcaster.broadcast(graph);
        sensorListener.clear();

        loopRate.sleep();
    }
}
