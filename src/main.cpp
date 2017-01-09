#include <angles/angles.h>
#include <boost/function.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <aruco_all_codes_tracker/DetectedMarker.h>
#include <aruco_all_codes_tracker/DetectedMarkerArray.h>

#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>

#include "csvlogger.h"
#include "markertracker.h"
#include "rvizhelpers.h"

#include "config.h"
#include "sensorlistener.h"
#include "transformgraph.h"
#include "transformgraphbroadcaster.h"

using MarkerArrayCallback = void(aruco_all_codes_tracker::DetectedMarkerArrayConstPtr);

int main(int argc, char** argv)
{
    // init ros
    ros::init(argc, argv, "atlas");
    ros::NodeHandle n;

    Config config("");
    SensorListener sensorListener(config);
    TransformGraph graph;

    //////////////////////////////////////
    ///      Main Loop
    //////////////////////////////////////
    ros::Rate loopRate(60);
    while (ros::ok())
    {
    }
}
