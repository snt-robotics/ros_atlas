/*
 * ATLAS - Cooperative sensing
 * Copyright (C) 2017  Paul KREMER
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
