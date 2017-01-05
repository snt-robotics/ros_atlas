#include "helpers.h"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cmath>
#include <iostream>

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

bool poseEq(const Pose& a, const Pose& b)
{
    bool test = quatEq(a.rot, b.rot) && vec3Eq(a.pos, b.pos);

    if (!test)
    {
        std::cout << "[EXPECTED] " << a << " [ACTUAL] " << b << std::endl;
    }

    return test;
}

// GTest main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "atlas_test", ros::init_options::NoRosout);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
