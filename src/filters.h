#pragma once

#include <eigen3/Eigen/Eigen>
#include <ros/time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <vector>

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
     */
    explicit ExplonentialMovingAverageFilter(double alpha);

    void addScalar(double scalar);
    void addVec3(const tf2::Vector3& vec);
    void addQuat(const tf2::Quaternion& quat);
    void reset();

    double scalar() const;
    tf2::Vector3 vec3() const;
    tf2::Quaternion quat() const;
    ros::Time timeOfLastValue() const;

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
};
