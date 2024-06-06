#include "range.h"
#include <gtest/gtest.h>

using namespace globe;

TEST(RangeTest, Map) {
    Range input_range(0.0, 10.0);
    Range output_range(0.0, 100.0);

    EXPECT_DOUBLE_EQ(Range::map(input_range, output_range, 5.0), 50.0);
    EXPECT_DOUBLE_EQ(Range::map(input_range, output_range, 2.5), 25.0);
    EXPECT_DOUBLE_EQ(Range::map(input_range, output_range, 7.5), 75.0);
}

TEST(RangeTest, MapDegenerateCase) {
    Range input_range(0.0, 0.0);
    Range output_range(0.0, 100.0);

    EXPECT_DOUBLE_EQ(Range::map(input_range, output_range, 1.0), 0.0);
}