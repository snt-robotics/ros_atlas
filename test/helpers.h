#pragma once

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <ostream>

namespace tf2
{
std::ostream& operator<<(std::ostream& os, const tf2::Vector3& vec);
std::ostream& operator<<(std::ostream& os, const tf2::Quaternion& quat);
std::ostream& operator<<(std::ostream& os, const tf2::Transform& transform);
}

bool scalarEq(double a, double b);
bool vec3Eq(const tf2::Vector3& a, const tf2::Vector3& b);
bool quatEq(const tf2::Quaternion& a, const tf2::Quaternion& b);
bool transfEq(const tf2::Transform& a, const tf2::Transform& b);
bool pathEq(const std::vector<std::string>& a, const std::vector<std::string>& b);
