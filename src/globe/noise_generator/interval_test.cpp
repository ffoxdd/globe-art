#include "interval.hpp"
#include <gtest/gtest.h>

using namespace globe;

TEST(IntervalTest, Map) {
    Interval input_range(0.0, 10.0);
    Interval output_range(0.0, 100.0);

    EXPECT_DOUBLE_EQ(Interval::map(input_range, output_range, 5.0), 50.0);
    EXPECT_DOUBLE_EQ(Interval::map(input_range, output_range, 2.5), 25.0);
    EXPECT_DOUBLE_EQ(Interval::map(input_range, output_range, 7.5), 75.0);
}

TEST(IntervalTest, MapDegenerateCase) {
    Interval input_range(0.0, 0.0);
    Interval output_range(0.0, 100.0);

    EXPECT_DOUBLE_EQ(Interval::map(input_range, output_range, 1.0), 0.0);
}