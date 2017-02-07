#include "helpers.h"

#include "../src/helpers.h"
#include "../src/kalmanfilter.h"

TEST(Kalman, simple)
{
    double dt = 1e-6;

    // State transition function
    FunctionHandle f = [dt](Eigen::VectorXd stateSpace) -> Eigen::MatrixXd {
        // calculates the state update
        //        Eigen::VectorXd x(2);
        //        x << stateSpace[0] + dt * stateSpace[1],
        //            stateSpace[1];

        return stateSpace;
    };

    // State Jacobian function
    FunctionHandle F = [dt](Eigen::VectorXd stateSpace) -> Eigen::MatrixXd {
        UNUSED(stateSpace)

        // states: 2
        // position and velocity
        // ->F is of size 2x2
        Eigen::MatrixXd A(2, 2);
        A << 1, dt, //
            0, 1;

        return A;
    };

    // Observation function
    FunctionHandle h = [dt](Eigen::VectorXd observations) -> Eigen::MatrixXd {
        // sensors: 1
        Eigen::VectorXd x(1);
        x << observations[0];
        return x;
    };

    // Observer Jacobian function
    FunctionHandle H = [dt](Eigen::VectorXd observations) -> Eigen::MatrixXd {
        UNUSED(observations)

        // maps the observations to the states
        // here we map the observation to state "position" (state 0)
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

    // feed the filter with 20 observations
    for (int i = 0; i < 20; ++i)
        ekf.update(z);

    // we expect the position state of the filter to converge to position 100
    // we also expect the velocity to be zero after some time
    Eigen::VectorXd result(2);
    result << 100, 0;

    ASSERT_TRUE(vecEq(result, ekf.stateVector()));
}

//TEST(Kalman, simpleFusion)
//{
//    double dt = 1e-6;

//    // State transition function
//    FunctionHandle f = [dt](Eigen::VectorXd stateSpace) -> Eigen::MatrixXd {
//        // calculates the state update
//        //        Eigen::VectorXd x(2);
//        //        x << stateSpace[0] + dt * stateSpace[1],
//        //            stateSpace[1];

//        return stateSpace;
//    };

//    // State Jacobian function
//    FunctionHandle F = [dt](Eigen::VectorXd stateSpace) -> Eigen::MatrixXd {
//        UNUSED(stateSpace)

//        // states: 2
//        // position and velocity
//        // ->F is of size 2x2
//        Eigen::MatrixXd A(2, 2);
//        A << 1, dt, //
//            0, 1;

//        return A;
//    };

//    // Observation function
//    FunctionHandle h = [dt](Eigen::VectorXd observations) -> Eigen::MatrixXd {
//        // observations: 2 (position and position)
//        return observations;
//    };

//    // Observer Jacobian function
//    FunctionHandle H = [dt](Eigen::VectorXd observations) -> Eigen::MatrixXd {
//        UNUSED(observations)

//        // maps the observations to the states
//        // here we map the observation to state "position" (state 0)
//        // observations: 2
//        // states: 2
//        // ->H is of size 2x2
//        Eigen::MatrixXd H(2, 2);
//        H << 1, 0,
//            1, 0; // we observe state 0 with both sensors

//        return H;
//    };

//    KalmanFilter ekf(2, 2, 0.2, 0.068 /*trustworthy observations*/, f, h, F, H);

//    // observations
//    Eigen::VectorXd z(2);
//    z << 101, 99;

//    // feed the filter with 20 observations
//    for (int i = 0; i < 200; ++i)
//        ekf.update(z);

//    // we expect the position state of the filter to converge to position 100
//    // we also expect the velocity to be zero after some time
//    Eigen::VectorXd result(2);
//    result << 100, 0;

//    ASSERT_TRUE(vecEq(result, ekf.stateVector()));
//}
