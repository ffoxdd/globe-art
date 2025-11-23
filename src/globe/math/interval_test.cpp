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

TEST(IntervalTest, MidpointReturnsMiddleValue) {
    Interval interval(0.0, 10.0);
    EXPECT_DOUBLE_EQ(interval.midpoint(), 5.0);
}

TEST(IntervalTest, MidpointWorksWithNegativeValues) {
    Interval interval(-10.0, 10.0);
    EXPECT_DOUBLE_EQ(interval.midpoint(), 0.0);
}

TEST(IntervalTest, MidpointWorksWithNegativeRange) {
    Interval interval(-20.0, -10.0);
    EXPECT_DOUBLE_EQ(interval.midpoint(), -15.0);
}

TEST(IntervalTest, MidpointOfZeroWidthInterval) {
    Interval interval(5.0, 5.0);
    EXPECT_DOUBLE_EQ(interval.midpoint(), 5.0);
}

TEST(IntervalTest, OverlapsWhenIntervalsOverlap) {
    Interval a(0.0, 10.0);
    Interval b(5.0, 15.0);

    EXPECT_TRUE(a.overlaps(b));
    EXPECT_TRUE(b.overlaps(a));
}

TEST(IntervalTest, OverlapsWhenOneContainsOther) {
    Interval a(0.0, 10.0);
    Interval b(2.0, 8.0);

    EXPECT_TRUE(a.overlaps(b));
    EXPECT_TRUE(b.overlaps(a));
}

TEST(IntervalTest, OverlapsWhenIntervalsAreSame) {
    Interval a(0.0, 10.0);
    Interval b(0.0, 10.0);

    EXPECT_TRUE(a.overlaps(b));
}

TEST(IntervalTest, OverlapsWhenIntervalsTouchAtBoundary) {
    Interval a(0.0, 10.0);
    Interval b(10.0, 20.0);

    EXPECT_TRUE(a.overlaps(b));
    EXPECT_TRUE(b.overlaps(a));
}

TEST(IntervalTest, DoesNotOverlapWhenIntervalsAreSeparated) {
    Interval a(0.0, 10.0);
    Interval b(15.0, 20.0);

    EXPECT_FALSE(a.overlaps(b));
    EXPECT_FALSE(b.overlaps(a));
}

TEST(IntervalTest, DoesNotOverlapWhenIntervalIsBeforeOther) {
    Interval a(0.0, 5.0);
    Interval b(10.0, 20.0);

    EXPECT_FALSE(a.overlaps(b));
    EXPECT_FALSE(b.overlaps(a));
}