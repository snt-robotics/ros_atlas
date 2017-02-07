#include "helpers.h"

#include "../src/filters.h"

TEST(Filters, weightedMeanVec3)
{
    WeightedMean filter;
    filter.addVec3({ 1, 1, 1 }, 0.5);
    filter.addVec3({ 1, 3, 1 }, 0.5);

    auto result   = filter.weightedMeanVec3();
    auto expected = tf2::Vector3{ 1, 2, 1 };
    ASSERT_EQ(expected, result);

    filter.reset();

    filter.addVec3({ 1, 1, 1 }, 0.0);
    filter.addVec3({ 1, 3, 1 }, 0.5);

    result   = filter.weightedMeanVec3();
    expected = tf2::Vector3{ 1, 3, 1 };
    ASSERT_EQ(expected, result);
}

TEST(Filters, weightedMeanQuatSimple)
{
    WeightedMean filter;
    filter.addQuat({ 0, 0, 0, 1 }, 0.5);
    auto result   = filter.weightedMeanQuat();
    auto expected = tf2::Quaternion{ 0, 0, 0, 1 };

    ASSERT_EQ(expected, result);
}

TEST(Filters, weightedMeanQuat)
{
    WeightedMean filter;
    filter.addQuat({ 0, 0, 0, 1 }, 0.5);
    filter.addQuat({ 0, 0, 0, 1 }, 0.5);
    auto result   = filter.weightedMeanQuat();
    auto expected = tf2::Quaternion{ 0, 0, 0, 1 };

    ASSERT_EQ(expected, result);
}

TEST(Filters, zeroWeight)
{
    WeightedMean filter;
    filter.addVec3({ 1, 1, 1 }, 0.0);
    auto result   = filter.weightedMeanVec3();
    auto expected = tf2::Vector3{ 0, 0, 0 };

    ASSERT_EQ(expected, result);
}

TEST(Filters, expMovingAverage)
{
    ExplonentialMovingAverage filter(0.5);

    // the filter is not initialized yet
    // we expect it to return {1,1,1}
    // otherwise it would return {0.5,0.5,0.5}
    filter.addVec3({ 1, 1, 1 });
    auto result   = filter.vec3();
    auto expected = tf2::Vector3{ 1, 1, 1 };
    ASSERT_EQ(expected, result);

    // the filter is initialized
    // we feed it with zero and check if
    // the values match
    filter.addVec3({ 0, 0, 0 });
    result   = filter.vec3();
    expected = tf2::Vector3{ 0.5, 0.5, 0.5 };
    ASSERT_EQ(expected, result);

    //
    filter.addVec3({ 0, 0, 0 });
    result   = filter.vec3();
    expected = tf2::Vector3{ 0.25, 0.25, 0.25 };
    ASSERT_EQ(expected, result);

    // reset test
    filter.reset();
    filter.addVec3({ 1, 1, 1 });
    result   = filter.vec3();
    expected = tf2::Vector3{ 1, 1, 1 };
    ASSERT_EQ(expected, result);
}
