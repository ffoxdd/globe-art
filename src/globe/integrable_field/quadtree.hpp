#ifndef GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_QUADTREE_H_
#define GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_QUADTREE_H_

#include "../scalar_field/interval.hpp"
#include <memory>
#include <array>
#include <functional>

namespace globe {

template<typename T>
class Quadtree {
 public:
    Quadtree(
        Interval x_range,
        Interval y_range,
        size_t max_depth,
        std::function<T(const Interval&, const Interval&)> leaf_initializer
    );

    bool is_leaf() const {
        return !_children[0];
    }

    const Interval& x_range() const {
        return _x_range;
    }

    const Interval& y_range() const {
        return _y_range;
    }

    const T& data() const {
        return _data;
    }

    T& data() {
        return _data;
    }

    const Quadtree& child(size_t i) const {
        return *_children[i];
    }

    Quadtree& child(size_t i) {
        return *_children[i];
    }

    const std::array<std::unique_ptr<Quadtree>, 4>& children() const {
        return _children;
    }

 private:
    Interval _x_range;
    Interval _y_range;
    T _data;
    std::array<std::unique_ptr<Quadtree>, 4> _children;

    void subdivide(size_t depth, size_t max_depth, std::function<T(const Interval&, const Interval&)> leaf_initializer);
};

template<typename T>
Quadtree<T>::Quadtree(
    Interval x_range,
    Interval y_range,
    size_t max_depth,
    std::function<T(const Interval&, const Interval&)> leaf_initializer
) :
    _x_range(x_range),
    _y_range(y_range),
    _data() {

    subdivide(0, max_depth, leaf_initializer);
}

template<typename T>
void Quadtree<T>::subdivide(
    size_t depth,
    size_t max_depth,
    std::function<T(const Interval&, const Interval&)> leaf_initializer
) {
    if (depth >= max_depth) {
        _data = leaf_initializer(_x_range, _y_range);
        return;
    }

    _children[0] = std::make_unique<Quadtree>(
        _x_range.lower_half(),
        _y_range.lower_half(),
        max_depth - depth - 1,
        leaf_initializer
    );

    _children[1] = std::make_unique<Quadtree>(
        _x_range.upper_half(),
        _y_range.lower_half(),
        max_depth - depth - 1,
        leaf_initializer
    );

    _children[2] = std::make_unique<Quadtree>(
        _x_range.lower_half(),
        _y_range.upper_half(),
        max_depth - depth - 1,
        leaf_initializer
    );

    _children[3] = std::make_unique<Quadtree>(
        _x_range.upper_half(),
        _y_range.upper_half(),
        max_depth - depth - 1,
        leaf_initializer
    );
}

}

#endif //GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_QUADTREE_H_
