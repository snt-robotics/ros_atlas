#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <ostream>

#define UNUSED(x) (void)(x);

// tf2 print helpers
namespace tf2
{
std::ostream& operator<<(std::ostream& os, const tf2::Vector3& vec);
std::ostream& operator<<(std::ostream& os, const tf2::Quaternion& quat);
std::ostream& operator<<(std::ostream& os, const tf2::Transform& transform);
}

struct Pose
{
    Pose() {}
    Pose(const tf2::Vector3& pos, const tf2::Quaternion& rot)
        : pos(pos)
        , rot(rot)
    {
    }

    tf2::Vector3 pos;
    tf2::Quaternion rot = tf2::Quaternion::getIdentity();
};

std::ostream& operator<<(std::ostream& os, const Pose& pose);

inline bool hasNanValues(const tf2::Vector3& pos)
{
    return isnan(pos.x()) || isnan(pos.y()) || isnan(pos.z());
}
