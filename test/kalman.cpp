#include "helpers.h"

#include "../src/kalmanfilter.h"

TEST(Kalman, simple)
{

    FunctionHandle f = [](Eigen::VectorXd stateSpace, double dt) -> Eigen::MatrixXd {
        // states: 2
        // ->F is of size 2x2
        //
        // simple integrator model
        // 1 dt
        // 0 1
        Eigen::MatrixXd A(2, 2);
        A << 1, dt, //
            0, 1;

        return A * stateSpace;
    };

    FunctionHandle h = [](Eigen::VectorXd observations, double dt) -> Eigen::MatrixXd {
        // observations: 1
        // states: 2
        // ->H is of size 1x2
        Eigen::MatrixXd H(1, 2);
        H << 1, 0;

        return observations * H;
    };

    KalmanFilter ekf(2, 1, 1, 1, f, h);
    Eigen::VectorXd z(1); // observations
    z << 100;

    for (int i = 0; i < 10000; ++i)
        ekf.update(1, z);

    Eigen::VectorXd result(2);
    result << 1, 0;

    ASSERT_TRUE(vecEq(result, ekf.stateVector()));
}
