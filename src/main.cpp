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

    // load the config
    Config config(configFile);

    // print the config
    config.dump();

    // init the rest
    SensorListener sensorListener(config);
    TransformGraph graph(config);
    TransformGraphBroadcaster broadcaster(config);

    //////////////////////////////////////
    ///      Main Loop
    //////////////////////////////////////
    ros::Rate loopRate(config.options().loopRate);
    ros::Time lastDump;

    while (ros::ok())
    {
        graph.update(sensorListener);
        broadcaster.broadcast(graph);
        sensorListener.clear();

        // save the graph if required
        if (!config.options().dbgGraphFilename.empty() && config.options().dbgGraphInterval > 0.0)
        {
            if (ros::Time::now() - lastDump > ros::Duration(config.options().dbgGraphInterval))
            {
                lastDump = ros::Time::now();
                graph.save(config.options().dbgGraphFilename);
            }
        }

        loopRate.sleep();
        ros::spinOnce();
    }
}
