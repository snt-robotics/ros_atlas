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

#pragma once

#include <eigen3/Eigen/Eigen>
#include <ros/time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <vector>

#include "helpers.h"

/**
 * @brief The WeightedMean class
 */
class WeightedMean
{
public:
    /**
     * @brief WeightedMean
     */
    WeightedMean();

    void addVec3(const tf2::Vector3& vec, double weight);
    void addQuat(const tf2::Quaternion& quat, double weight);
    void reset();

    /**
     * @brief weightedMeanVec3
     * @return The weighted mean of all vectors
     */
    tf2::Vector3 weightedMeanVec3() const;

    /**
     * @brief weightedMeanQuat
     * @return The weighted mean of all quaternions
     */
    tf2::Quaternion weightedMeanQuat() const;

private:
    Eigen::Vector3d m_vectorWeightedSum = { 0, 0, 0 };
    Eigen::Matrix4Xd m_quats;

    double m_vectorWeights = 0.0;
};

/**
 * @brief The ExplonentialMovingAverage class
 */
class ExplonentialMovingAverageFilter
{
public:
    /**
     * @brief ExplonentialMovingAverage
     */
    ExplonentialMovingAverageFilter();

    /**
     * @brief ExplonentialMovingAverage
     * @param alpha is the exponential factor. Lower alpha means slower filter (or higher time constant).
     * @param timeout tells the filter to reinitialize after a given amount of time without data
     */
    explicit ExplonentialMovingAverageFilter(double alpha, ros::Duration timeout = ros::Duration(0));

    /**
     * @brief addScalar
     * @param scalar: The scalar to filter
     */
    void addScalar(double scalar);

    /**
     * @brief addVec3
     * @param vec: The vector to filter
     */
    void addVec3(const tf2::Vector3& vec);

    /**
     * @brief addQuat
     * @param quat: The quaternion to filter
     */
    void addQuat(const tf2::Quaternion& quat);

    /**
     * @brief addPose
     * @param pose
     */
    void addPose(const Pose& pose);

    /**
     * @brief reset
     * Resets the scalar, quaternion and vector of the filter
     * They will be reinitialized once the filter receives new data
     */
    void reset();

    /**
     * @brief scalar
     * @return the scalar of the filter
     */
    double scalar() const;

    /**
     * @brief vec3
     * @return the vector of the filter
     */
    tf2::Vector3 vec3() const;

    /**
     * @brief quat
     * @return the quaternion of the filter
     */
    tf2::Quaternion quat() const;

    /**
     * @brief pose
     * @return the filtered pose
     */
    Pose pose() const;

    /**
     * @brief timeOfLastValue
     * @return returns the time the filter received its last value
     */
    ros::Time timeOfLastValue() const;

    /**
     * @brief setTimeout
     * @param timeout tells the filter to reinitialize after a given amount of time without data
     */
    void setTimeout(const ros::Duration& timeout);

    /**
     * @brief alpha
     * @return the exponential weighting factor
     */
    double alpha() const;

    /**
     * @brief setAlpha
     * @param set the exponential weighting factor alpha of the filter
     */
    void setAlpha(double alpha);

protected:
    void checkReinit();

private:
    double m_alpha = 0.05; // exponential constant

    // the accumulators
    double m_scalarAccu        = 0.0;
    tf2::Vector3 m_vectorAccu  = { 0, 0, 0 };
    tf2::Quaternion m_quatAccu = tf2::Quaternion::getIdentity();

    // we keep track of the time of the last value
    // in order to reset the filter if it's too old
    ros::Time m_timeOfLastValue;

    bool m_scalarInitialized = false;
    bool m_vecInitialized    = false;
    bool m_quatInitialized   = false;
    ros::Duration m_timeout;
};
