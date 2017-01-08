#pragma once

#include <eigen3/Eigen/Eigen>
#include <functional>

/**
 * @arg x: vector of values (can be state space or measurements)
 * @arg dt: elapsed time
 * @return Returns the predicted state space
 */
using FunctionHandle = std::function<Eigen::MatrixXd(Eigen::VectorXd x, double dt)>;

/**
 * @brief The KalmanFilter class
 * Implements an extended Kalman filter
 * Implementation based on Yi Cao's EKF matlab file:
 * https://nl.mathworks.com/matlabcentral/fileexchange/18189-learning-the-extended-kalman-filter
 */
class KalmanFilter
{
public:
    /**
     * @brief KalmanFilter
     * @param numberOfState
     * @param q: standard variance of the process
     * @param r: standard variance of the measurements
     */
    KalmanFilter(int numberOfStates, int numberOfObservations, double q, double r, const FunctionHandle& f, const FunctionHandle& h);

    void update(double dt, Eigen::VectorXd measurements);
    Eigen::VectorXd stateVector() const;

protected:
    /**
     * @brief jacobian calculates the Jaccobian matrix of a given function
     * @param func
     * @param x
     * @param dt
     * @return
     */
    Eigen::MatrixXd jacobian(const FunctionHandle& func, Eigen::VectorXd x, double dt);

private:
    Eigen::MatrixXd m_I; // identity
    Eigen::MatrixXd m_Q; // process covariance
    Eigen::MatrixXd m_R; // measurements covariance
    Eigen::MatrixXd m_P; // state covariance
    Eigen::VectorXd m_x; // state space
    FunctionHandle m_f; // f(x) (state)
    FunctionHandle m_h; // h(x) (sensor)
};
