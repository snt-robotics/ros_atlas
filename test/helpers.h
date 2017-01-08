#pragma once

#include "../src/helpers.h"

#include <eigen3/Eigen/Eigen>
#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

bool scalarEq(double a, double b);
bool vec3Eq(const tf2::Vector3& a, const tf2::Vector3& b);
bool vecEq(const Eigen::VectorXd& a, const Eigen::VectorXd& b);
bool quatEq(const tf2::Quaternion& a, const tf2::Quaternion& b);
bool transfEq(const tf2::Transform& a, const tf2::Transform& b);
bool pathEq(const std::vector<std::string>& a, const std::vector<std::string>& b);
bool poseEq(const Pose& a, const Pose& b);
