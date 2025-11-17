#include "quadtree.hpp"
#include <gtest/gtest.h>

using namespace globe;

TEST(QuadtreeTest, RootNodeHasCorrectRange) {
    Quadtree<int> tree(
        Interval(0, 10),
        Interval(0, 10),
        0,
        [](const Interval&, const Interval&) { return 0; }
    );

    EXPECT_EQ(tree.x_range().low(), 0);
    EXPECT_EQ(tree.x_range().high(), 10);
    EXPECT_EQ(tree.y_range().low(), 0);
    EXPECT_EQ(tree.y_range().high(), 10);
}

TEST(QuadtreeTest, MaxDepthZeroCreatesLeafNode) {
    Quadtree<int> tree(
        Interval(0, 10),
        Interval(0, 10),
        0,
        [](const Interval&, const Interval&) { return 42; }
    );

    EXPECT_TRUE(tree.is_leaf());
    EXPECT_EQ(tree.data(), 42);
}

TEST(QuadtreeTest, MaxDepthOneCreatesFourChildren) {
    Quadtree<int> tree(
        Interval(0, 10),
        Interval(0, 10),
        1,
        [](const Interval&, const Interval&) { return 0; }
    );

    EXPECT_FALSE(tree.is_leaf());
    EXPECT_TRUE(tree.children()[0] != nullptr);
    EXPECT_TRUE(tree.children()[1] != nullptr);
    EXPECT_TRUE(tree.children()[2] != nullptr);
    EXPECT_TRUE(tree.children()[3] != nullptr);
}

TEST(QuadtreeTest, ChildrenHaveCorrectRanges) {
    Quadtree<int> tree(
        Interval(0, 10),
        Interval(0, 10),
        1,
        [](const Interval&, const Interval&) { return 0; }
    );

    EXPECT_EQ(tree.child(0).x_range().low(), 0);
    EXPECT_EQ(tree.child(0).x_range().high(), 5);
    EXPECT_EQ(tree.child(0).y_range().low(), 0);
    EXPECT_EQ(tree.child(0).y_range().high(), 5);

    EXPECT_EQ(tree.child(1).x_range().low(), 5);
    EXPECT_EQ(tree.child(1).x_range().high(), 10);
    EXPECT_EQ(tree.child(1).y_range().low(), 0);
    EXPECT_EQ(tree.child(1).y_range().high(), 5);

    EXPECT_EQ(tree.child(2).x_range().low(), 0);
    EXPECT_EQ(tree.child(2).x_range().high(), 5);
    EXPECT_EQ(tree.child(2).y_range().low(), 5);
    EXPECT_EQ(tree.child(2).y_range().high(), 10);

    EXPECT_EQ(tree.child(3).x_range().low(), 5);
    EXPECT_EQ(tree.child(3).x_range().high(), 10);
    EXPECT_EQ(tree.child(3).y_range().low(), 5);
    EXPECT_EQ(tree.child(3).y_range().high(), 10);
}

TEST(QuadtreeTest, LeafInitializerCalledForEachLeaf) {
    int call_count = 0;
    Quadtree<int> tree(
        Interval(0, 10),
        Interval(0, 10),
        1,
        [&call_count](const Interval&, const Interval&) {
            call_count++;
            return call_count;
        }
    );

    EXPECT_EQ(call_count, 4);
    EXPECT_EQ(tree.child(0).data(), 1);
    EXPECT_EQ(tree.child(1).data(), 2);
    EXPECT_EQ(tree.child(2).data(), 3);
    EXPECT_EQ(tree.child(3).data(), 4);
}

TEST(QuadtreeTest, LeafInitializerReceivesCorrectRanges) {
    std::vector<std::pair<Interval, Interval>> ranges;
    Quadtree<int> tree(
        Interval(0, 10),
        Interval(0, 10),
        1,
        [&ranges](const Interval& x, const Interval& y) {
            ranges.push_back({x, y});
            return 0;
        }
    );

    ASSERT_EQ(ranges.size(), 4);

    EXPECT_EQ(ranges[0].first.low(), 0);
    EXPECT_EQ(ranges[0].first.high(), 5);
    EXPECT_EQ(ranges[0].second.low(), 0);
    EXPECT_EQ(ranges[0].second.high(), 5);

    EXPECT_EQ(ranges[1].first.low(), 5);
    EXPECT_EQ(ranges[1].first.high(), 10);
    EXPECT_EQ(ranges[1].second.low(), 0);
    EXPECT_EQ(ranges[1].second.high(), 5);

    EXPECT_EQ(ranges[2].first.low(), 0);
    EXPECT_EQ(ranges[2].first.high(), 5);
    EXPECT_EQ(ranges[2].second.low(), 5);
    EXPECT_EQ(ranges[2].second.high(), 10);

    EXPECT_EQ(ranges[3].first.low(), 5);
    EXPECT_EQ(ranges[3].first.high(), 10);
    EXPECT_EQ(ranges[3].second.low(), 5);
    EXPECT_EQ(ranges[3].second.high(), 10);
}

TEST(QuadtreeTest, MaxDepthTwoCreatesCorrectStructure) {
    Quadtree<int> tree(
        Interval(0, 8),
        Interval(0, 8),
        2,
        [](const Interval&, const Interval&) { return 0; }
    );

    EXPECT_FALSE(tree.is_leaf());
    EXPECT_FALSE(tree.child(0).is_leaf());
    EXPECT_TRUE(tree.child(0).child(0).is_leaf());
}

TEST(QuadtreeTest, WorksWithNonIntegerTypes) {
    struct CustomData {
        double value;
        std::string label;
    };

    Quadtree<CustomData> tree(
        Interval(0, 1),
        Interval(0, 1),
        1,
        [](const Interval& x, const Interval&) {
            return CustomData{x.low(), "test"};
        }
    );

    EXPECT_EQ(tree.child(0).data().value, 0.0);
    EXPECT_EQ(tree.child(0).data().label, "test");
    EXPECT_EQ(tree.child(1).data().value, 0.5);
}
