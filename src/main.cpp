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

using MarkerArrayCallback = void(aruco_all_codes_tracker::DetectedMarkerArrayConstPtr);

int main(int argc, char** argv)
{
    // init ros
    ros::init(argc, argv, "atlas");
    ros::NodeHandle n;

    ros::Rate loopRate(100);

    tf::TransformBroadcaster tfBroadCaster;
    tf::TransformListener tfListener;

    MarkerTracker tracker;
    RvizHelpers rviz(n);

    CSVLogger logger;

    //////////////////////////////////////
    /// Callback Marker Array
    /// from the aruco detector
    //////////////////////////////////////
    boost::function<MarkerArrayCallback>
        callbackMarkers
        = [&tfListener, &tracker, &logger](const aruco_all_codes_tracker::DetectedMarkerArrayConstPtr& markerArray) {
              for (const aruco_all_codes_tracker::DetectedMarker& marker : markerArray->markers)
              {
                  // the marker id
                  auto markerTf = marker.transform.child_frame_id;
                  // find transform map->marker
                  std::string error;
                  if (tfListener.canTransform("map", markerTf, ros::Time(0), &error))
                  {
                      tf::StampedTransform transf;
                      tfListener.lookupTransform("map", markerTf, ros::Time(0), transf);
                      transf.stamp_ = ros::Time::now();

                      auto pos  = transf.getOrigin();
                      auto quad = transf.getRotation();

                      logger.log(markerTf, marker.id, marker.distance, transf);
                  }
              }
          };

    auto subscriberMarkers = n.subscribe<MarkerArrayCallback>("/aruco_tracker/detected_markers", 1, callbackMarkers);

    //////////////////////////////////////
    ///      Main Loop
    ///
    //////////////////////////////////////
    while (ros::ok())
    {
        tf::StampedTransform transf;

        transf.setIdentity();
        transf.setOrigin({ 0.185, 0, -0.03 });
        transf.setRotation(tf::Quaternion({ 0, 1, 0 }, angles::from_degrees(90 + 28)) * tf::Quaternion({ 0, 0, 1 }, angles::from_degrees(270)));

        transf.child_frame_id_ = "Ardrone2SimpleLinModel_HASHMARK_0/ardrone_base_frontcam_";
        transf.frame_id_       = "Ardrone2SimpleLinModel_HASHMARK_0/base_link";
        transf.stamp_          = ros::Time::now();
        tfBroadCaster.sendTransform(transf);

        transf.child_frame_id_ = "Ardrone2SimpleLinModel_HASHMARK_1/ardrone_base_frontcam_";
        transf.frame_id_       = "Ardrone2SimpleLinModel_HASHMARK_1/base_link";
        transf.stamp_          = ros::Time::now();
        tfBroadCaster.sendTransform(transf);

        std::vector<std::string> frames;
        tfListener.getFrameStrings(frames);

        // filter

        for (int drone = 0; drone < 2; ++drone)
            for (int i = 0; i < 9; ++i)
            {
                std::stringstream ss;
                ss << "ardrone" << drone << "_frontcam/" << i;
                auto markerId = ss.str();

                std::string error;
                std::string error2;
                if (tfListener.canTransform("map", markerId, ros::Time(0), &error))
                {
                    tfListener.lookupTransform("map", markerId, ros::Time(0), transf);

                    tracker.feed(i, transf);
                }
                else
                {
                    //ROS_INFO("%s |||| %s", error.c_str(), error2.c_str());
                }
            }

        tracker.guessAll();

        // publish markers in Rviz
        for (int id : tracker.getKnownMarkers())
        {
            auto transf = tracker.getTransform(id);
            rviz.publishMarker(id, transf);

            //std::cout << "ID: " << id << "| X: " << transf.getOrigin().x() << " Y: " << transf.getOrigin().y() << " Z: " << transf.getOrigin().z() << std::endl;
            //std::cout << transf.getOrigin().x() << ";" << transf.getOrigin().y() << ";" << transf.getOrigin().z() << std::endl;
        }

        ros::spinOnce();
        loopRate.sleep();
    }

    // enter event loop
    //ros::spin();
}
