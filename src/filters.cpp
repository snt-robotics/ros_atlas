#include "filters.h"

/**
 * @brief WeightedMean
 */

WeightedMean::WeightedMean()
{
}

void WeightedMean::addVec3(const tf2::Vector3& vec, double weight)
{
    m_vectorWeightedSum += Eigen::Vector3d{ vec.x(), vec.y(), vec.z() } * weight;
    m_vectorWeights += weight;
}

void WeightedMean::addQuat(const tf2::Quaternion& quat, double weight)
{
    // quaternion weighted sum
    m_quats.conservativeResize(4, m_quats.cols() + 1);
    m_quats.col(m_quats.cols() - 1) = weight * Eigen::Vector4d{
        quat.x(),
        quat.y(),
        quat.z(),
        quat.w()
    };
}

void WeightedMean::reset()
{
    m_vectorWeightedSum = { 0, 0, 0 };
    m_vectorWeights     = 0.0;
    m_quats             = Eigen::Matrix4Xd();
}

tf2::Vector3 WeightedMean::weightedMeanVec3() const
{
    if (m_vectorWeights == 0.0)
        return { 0, 0, 0 };

    auto average = m_vectorWeightedSum / m_vectorWeights;
    return { average[0], average[1], average[2] };
}

tf2::Quaternion WeightedMean::weightedMeanQuat() const
{
    // quaternion interpolation matrix
    // calculations based on http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf

    // solve the eigenproblem
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4Xd> solver(m_quats * m_quats.transpose());

    // find largest eigenvalue
    int index     = 0;
    double maxVal = -1.0;

    for (int i = 0; i < solver.eigenvalues().rows(); ++i)
    {
        if (std::abs(solver.eigenvalues()[i]) > maxVal)
        {
            maxVal = std::abs(solver.eigenvalues()[i]);
            index  = i;
        }
    }

    // the eigenvector corresponding to the largest eigenvalue
    // is the weighted average of the quaternion
    auto eigenvec = solver.eigenvectors().col(index);
    return { eigenvec[0], eigenvec[1], eigenvec[2], eigenvec[3] };
}

/**
 * @brief ExplonentialMovingAverage
 */

ExplonentialMovingAverageFilter::ExplonentialMovingAverageFilter()
{
}

ExplonentialMovingAverageFilter::ExplonentialMovingAverageFilter(double alpha)
    : m_alpha(alpha)
{
}

void ExplonentialMovingAverageFilter::addScalar(double scalar)
{
    if (m_scalarInitialized)
    {
        m_scalarAccu = (m_alpha * scalar) + (1.0 - m_alpha) * m_scalarAccu;
    }
    else
    {
        m_scalarAccu        = scalar;
        m_scalarInitialized = true;
    }

    m_timeOfLastValue = ros::Time::now();
}

void ExplonentialMovingAverageFilter::addVec3(const tf2::Vector3& vec)
{
    if (m_vecInitialized)
    {
        m_vectorAccu = (m_alpha * vec) + (1.0 - m_alpha) * m_vectorAccu;
    }
    else
    {
        m_vectorAccu     = vec;
        m_vecInitialized = true;
    }

    m_timeOfLastValue = ros::Time::now();
}

void ExplonentialMovingAverageFilter::addQuat(const tf2::Quaternion& quat)
{
    if (m_quatInitialized)
    {
        m_quatAccu = m_quatAccu.slerp(quat, m_alpha);
    }
    else
    {
        m_quatAccu        = quat;
        m_quatInitialized = true;
    }

    m_timeOfLastValue = ros::Time::now();
}

void ExplonentialMovingAverageFilter::reset()
{
    m_scalarInitialized = false;
    m_quatInitialized   = false;
    m_vecInitialized    = false;
}

double ExplonentialMovingAverageFilter::scalar() const
{
    return m_scalarAccu;
}

tf2::Vector3 ExplonentialMovingAverageFilter::vec3() const
{
    return m_vectorAccu;
}

tf2::Quaternion ExplonentialMovingAverageFilter::quat() const
{
    return m_quatAccu;
}

ros::Time ExplonentialMovingAverageFilter::timeOfLastValue() const
{
    return m_timeOfLastValue;
}
