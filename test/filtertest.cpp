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
