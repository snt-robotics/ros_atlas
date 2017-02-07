#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "atlas_test", ros::init_options::NoRosout);
    ros::start(); // required by ros::Timer
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
