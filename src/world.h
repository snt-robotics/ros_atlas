#pragma once

#include "config.h"
#include "sensorlistener.h"

#include <geometry_msgs/Pose.h>

struct WorldPose
{
    tf2::Quaternion rot;
    tf2::Vector3 pos;
    ros::Time timeStamp;
};

class World
{
public:
    World(const Config& config);

    void update(const FilteredSensorDataMap& sensorData);
    tf2::Transform entryPose(const std::string& entry);

private:
    // the entities' world pose
    std::map<std::string, WorldPose> m_worldPoses;

    // a bunch of estimates for a given entity
    // this data will be used to perform sensor fusion
    // entity - entity, pose
    std::map<std::string, std::map<std::string, WorldPose> > m_worldPoseEstimates;

    // world markers
    // i.e. markers whose pose is known in world space
    std::map<int, WorldPose> m_worldMarkers;
};
