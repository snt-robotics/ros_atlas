#include "kalmanfilter.h"

#include <assert.h>

KalmanFilter::KalmanFilter(int numberOfStates, int numberOfObservations, double q, double r, const FunctionHandle& f, const FunctionHandle& h, const FunctionHandle& F, const FunctionHandle& H)
    : m_f(f)
    , m_h(h)
    , m_F(F)
    , m_H(H)
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

    // initialize identity
    m_I.setIdentity(numberOfStates, numberOfStates);

    // matrix size check
    auto a = f(Eigen::VectorXd(numberOfStates));
    assert(a.rows() == numberOfStates);

    auto b = h(Eigen::VectorXd(numberOfObservations));
    assert(b.rows() == numberOfObservations);

    auto FM = F(Eigen::VectorXd(numberOfStates));
    auto HM = H(Eigen::VectorXd(numberOfObservations));

    assert(FM.rows() == numberOfStates && FM.cols() == numberOfStates);
    assert(HM.rows() == numberOfObservations && HM.cols() == numberOfStates);
}

void KalmanFilter::update(Eigen::VectorXd z)
{
    auto F = m_F(m_x);
    auto H = m_H(z);
    m_P    = F * m_P * F.transpose() + m_Q;
    auto G = (m_P * H.transpose() * (H * m_P * H.transpose() + m_R).inverse()).eval();
    m_x    = m_x + G * (z - m_h(m_x)).transpose();
    m_P    = (m_I - G * H) * m_P;
}

Eigen::VectorXd KalmanFilter::stateVector() const
{
    return m_x;
}
