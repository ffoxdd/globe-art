#include "gtest/gtest.h"
#include "math/interval_sampler.hpp"

using namespace globe;

TEST(IntervalSamplerTest, SamplesWithinBounds) {
    IntervalSampler sampler;
    Interval interval(2.5, 5.0);

    for (int i = 0; i < 1000; ++i) {
        double value = sampler.sample(interval);
        EXPECT_GE(value, interval.low());
        EXPECT_LT(value, interval.high());
    }
}

TEST(IntervalSamplerTest, WorksWithDegenerateInterval) {
    IntervalSampler sampler;
    Interval interval(1.0, 1.0);

    for (int i = 0; i < 10; ++i) {
        double value = sampler.sample(interval);
        EXPECT_DOUBLE_EQ(value, 1.0);
    }
}
