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
    m_Ppost.setIdentity(numberOfStates, numberOfStates);
    m_Ppost *= 0.1;

    // initialize state vector
    m_x.resize(numberOfStates);
    m_x.setZero();

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
    auto H = m_H(m_x);

    // predict
    m_x    = m_f(m_x);
    m_Ppre = F * m_Ppost * F.transpose() + m_Q;

    // update
    auto G = (m_Ppre * H.transpose() * (H * m_Ppre * H.transpose() + m_R).inverse()).eval();

    m_x += G * (m_h(z) - m_h(m_x));
    m_Ppost = (m_I - G * H) * m_Ppre;
}

Eigen::VectorXd KalmanFilter::stateVector() const
{
    return m_x;
}
