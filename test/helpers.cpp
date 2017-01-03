#include "helpers.h"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cmath>
#include <iostream>

namespace tf2
{
// Print helpers
::std::ostream& operator<<(std::ostream& os, const tf2::Vector3& vec)
{
    return os << "Vec3 [X:] " << vec.x() << " [Y:] " << vec.y() << "[Z:] " << vec.z();
}

::std::ostream& operator<<(std::ostream& os, const tf2::Quaternion& quat)
{
    return os << "Quad [X:] " << quat.x() << " [Y:] " << quat.y() << "[Z:] " << quat.z() << "[W:] " << quat.w();
}

::std::ostream& operator<<(std::ostream& os, const tf2::Transform& transform)
{
    return os << transform.getOrigin() << " " << transform.getRotation();
}
}

::std::ostream& operator<<(std::ostream& os, const std::vector<std::string> vec)
{
    for (const auto& str : vec)
        os << "\"" << str << "\" ";

    return os;
}

// Equality helpers
bool scalarEq(double a, double b)
{
    return std::abs(a - b) < 0.001;
}

bool vec3Eq(const tf2::Vector3& a, const tf2::Vector3& b)
{
    bool test = scalarEq(a.x(), b.x()) && scalarEq(a.y(), b.y()) && scalarEq(a.z(), b.z());

    if (!test)
    {
        std::cout << "[EXPECTED] " << a << " [ACTUAL] " << b << std::endl;
    }

    return test;
}

bool quatEq(const tf2::Quaternion& a, const tf2::Quaternion& b)
{
    bool test = scalarEq(a.x(), b.x()) && scalarEq(a.y(), b.y()) && scalarEq(a.z(), b.z()) && scalarEq(a.w(), b.w());

    if (!test)
    {
        std::cout << "[EXPECTED] " << a << " [ACTUAL] " << b << std::endl;
    }

    return test;
}

bool transfEq(const tf2::Transform& a, const tf2::Transform& b)
{
    bool test = quatEq(a.getRotation(), b.getRotation()) && vec3Eq(a.getOrigin(), b.getOrigin());

    if (!test)
    {
        std::cout << "[EXPECTED] " << a << " [ACTUAL] " << b << std::endl;
    }

    return test;
}

bool pathEq(const std::vector<std::string>& a, const std::vector<std::string>& b)
{
    bool test = (a == b);

    if (!test)
    {
        std::cout << "[EXPECTED] " << a << " [ACTUAL] " << b << std::endl;
    }

    return test;
}

// GTest main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "atlas_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
