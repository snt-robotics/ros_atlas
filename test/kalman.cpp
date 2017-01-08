#include "helpers.h"

#include "../src/kalmanfilter.h"

TEST(Kalman, simple)
{
    double dt = 1e-6;

    FunctionHandle f = [dt](Eigen::VectorXd stateSpace) -> Eigen::MatrixXd {
        Eigen::VectorXd x(2);
        x << stateSpace[0] + dt * stateSpace[1],
            stateSpace[1];
        return x;
    };

    // Jacobian
    FunctionHandle F = [dt](Eigen::VectorXd stateSpace) -> Eigen::MatrixXd {
        // states: 2
        // ->F is of size 2x2
        Eigen::MatrixXd A(2, 2);
        A << 1, dt, //
            0, 1;

        return A;
    };

    FunctionHandle h = [dt](Eigen::VectorXd observations) -> Eigen::MatrixXd {
        Eigen::VectorXd x(1);
        x << observations[0];
        return x;
    };

    FunctionHandle H = [dt](Eigen::VectorXd observations) -> Eigen::MatrixXd {
        // observations: 1
        // states: 2
        // ->H is of size 1x2
        Eigen::MatrixXd H(1, 2);
        H << 1, 0; // we observe state 0

        return H;
    };

    KalmanFilter ekf(2, 1, 0.2, 0.068 /*trustworthy observations*/, f, h, F, H);

    // observations
    Eigen::VectorXd z(1);
    z << 100;

    for (int i = 0; i < 2; ++i)
        ekf.update(z);

    Eigen::VectorXd result(2);
    result << 100, 0;

    ASSERT_TRUE(vecEq(result, ekf.stateVector()));
}
