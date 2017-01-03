#include "filters.h"

WeightedMean::WeightedMean()
{
}

void WeightedMean::addVec3(const tf2::Vector3& vec, double weight)
{
    m_vectors += Eigen::Vector3d{ vec.x(), vec.y(), vec.z() } * weight;
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

tf2::Vector3 WeightedMean::weightedMeanVec3() const
{
    if (m_vectorWeights == 0.0)
        return { 0, 0, 0 };

    auto average = m_vectors / m_vectorWeights;
    return { average[0], average[1], average[2] };
}

tf2::Quaternion WeightedMean::weightedMeanQuat() const
{
    // quaternion interpolation matrix
    // calculations based on http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf

    // solve the eigenproblem
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4Xd> solver(m_quats * m_quats.transpose());

    // find largest eigenvalue
    std::size_t index = 0;
    double maxVal     = -1.0;

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
