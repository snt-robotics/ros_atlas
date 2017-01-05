#include "helpers.h"

namespace tf2
{
// Print helpers
std::ostream& operator<<(std::ostream& os, const tf2::Vector3& vec)
{
    return os << "Vec3 [X:] " << vec.x() << " [Y:] " << vec.y() << "[Z:] " << vec.z();
}

std::ostream& operator<<(std::ostream& os, const tf2::Quaternion& quat)
{
    return os << "Quad [X:] " << quat.x() << " [Y:] " << quat.y() << "[Z:] " << quat.z() << "[W:] " << quat.w();
}

std::ostream& operator<<(std::ostream& os, const tf2::Transform& transform)
{
    return os << transform.getOrigin() << " " << transform.getRotation();
}
}

std::ostream& operator<<(std::ostream& os, const Pose& pose)
{
    return os << pose.pos << " " << pose.rot;
}
