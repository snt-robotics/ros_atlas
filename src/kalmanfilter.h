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
#include <functional>

/**
 * @arg x: vector of values (can be state space or measurements)
 * @return
 */
using FunctionHandle = std::function<Eigen::MatrixXd(Eigen::VectorXd x)>;

/**
 * @brief The KalmanFilter class
 * Implements an extended Kalman filter
 * Implementation based TinyEKF:
 * https://github.com/simondlevy/TinyEKF
 */
class KalmanFilter
{
public:
    /**
     * @brief KalmanFilter
     * @param numberOfStates
     * @param numberOfObservations
     * @param q: Standard variance of the process
     * @param r: Standard variance of the measurements
     * @param f: State transition function
     * @param h: Observation function
     * @param F: State Jacobian function
     * @param H: Observer Jacobian function
     */
    KalmanFilter(int numberOfStates, int numberOfObservations, double q, double r,
        const FunctionHandle& f, const FunctionHandle& h, const FunctionHandle& F, const FunctionHandle& H);

    void update(Eigen::VectorXd measurements);
    Eigen::VectorXd stateVector() const;

private:
    Eigen::MatrixXd m_I; // identity
    Eigen::MatrixXd m_Q; // process covariance
    Eigen::MatrixXd m_R; // measurements covariance
    Eigen::MatrixXd m_Ppre; // state covariance
    Eigen::MatrixXd m_Ppost; // state covariance
    Eigen::VectorXd m_x; // state space
    FunctionHandle m_f; // f(x) (state)
    FunctionHandle m_h; // h(x) (sensor)
    FunctionHandle m_F; // Jacobian (state)
    FunctionHandle m_H; // Jacobian (sensor)
};
