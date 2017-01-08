#include "kalmanfilter.h"

#include <assert.h>
#include <iostream>

#include <eigen3/Eigen/Cholesky>

KalmanFilter::KalmanFilter(int numberOfStates, int numberOfObservations, double q, double r, const FunctionHandle& f, const FunctionHandle& h)
    : m_f(f)
    , m_h(h)
{
    // initialize process covariance
    m_Q.setIdentity(numberOfStates, numberOfStates);
    m_Q *= q * q;

    // initialize measurements covariance
    m_R.setIdentity(numberOfObservations, numberOfObservations);
    m_R *= r * r;

    // initialize process covariance
    m_P.setIdentity(numberOfStates, numberOfStates);

    // initialize state vector
    m_x.resize(numberOfStates);
    m_x.setRandom();
    //m_x.setZero();

    // initialize identity
    m_I.setIdentity(numberOfStates, numberOfStates);

    // matrix size check
    auto a = f(Eigen::VectorXd(numberOfStates), 1);
    assert(a.cols() == numberOfObservations);

    auto F = jacobian(f, Eigen::VectorXd(numberOfStates), 1);
    auto H = jacobian(h, Eigen::VectorXd(numberOfObservations), 1);

    assert(F.rows() == numberOfStates && F.cols() == numberOfStates);
    assert(H.rows() == numberOfObservations && H.cols() == numberOfStates);
    //    assert(m_R.rows() == numberOfObservations && m_R.cols() == numberOfObservations);
    //    assert(m_Q.rows() == numberOfStates && m_Q.cols() == numberOfStates);
}

void KalmanFilter::update(double dt, Eigen::VectorXd z)
{
    //    // Model
    //    auto xk = m_f(m_x, dt); //+wk
    //    auto zk = m_h(z, dt); // +vk

    //    // Predict
    //    m_x = m_f(m_x, dt);

    //    auto A = jacobian(m_f, m_x, dt);
    //    auto H = jacobian(m_h, z, dt);

    //    m_P = A * m_P * A.transpose() + m_Q;

    //    // Update
    //    auto G = m_P * H.transpose() * (H * m_P * H.transpose() + m_R).inverse(); // Kalman gain
    //    m_x    = m_x + G * (zk - m_h(z, dt));
    //    m_P    = (m_I - G * H) * m_P;

    auto A = jacobian(m_f, m_x, dt);
    auto H = jacobian(m_h, z, dt);
    m_P    = A * m_P * A.transpose() + m_Q;

    auto G    = m_P * H.transpose() * (H * m_P * H.transpose() + m_R).inverse();
    auto tmp1 = m_h(m_x, dt);
    //auto tmp2 = ;
    m_x = m_x + G * (z - m_h(m_x, dt));
    m_P = (m_I - G * H) * m_P;
}

Eigen::VectorXd KalmanFilter::stateVector() const
{
    return m_x;
}

Eigen::MatrixXd KalmanFilter::jacobian(const FunctionHandle& func, Eigen::VectorXd x, double dt)
{
    // epsilon
    const double h = 10e-3;

    auto z = func(x, dt);
    auto n = std::max(x.rows(), x.cols());
    auto m = std::max(z.rows(), z.cols());
    auto A = Eigen::MatrixXd(n, m);

    for (auto k = 0; k < n; ++k)
    {
        // differentiate numerically
        auto x1     = x;
        x1[k]       = x[k] + h;
        auto assign = (x1 - x) / h;
        A.col(k)    = assign;
    }

    return A;
}
