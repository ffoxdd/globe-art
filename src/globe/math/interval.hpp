#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_RANGE_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_RANGE_H_

#include "../std_ext/ranges.hpp"
#include <CGAL/assertions.h>
#include <limits>
#include <concepts>
#include <ranges>
#include <algorithm>
#include <numeric>

namespace globe {

class Interval;

class Interval {
 public:
    Interval();
    Interval(double low, double high);
    template<RangeOf<double> DoubleRange> explicit Interval(const DoubleRange &range);

    [[nodiscard]] double low() const;
    [[nodiscard]] double high() const;
    [[nodiscard]] double measure() const;
    [[nodiscard]] double midpoint() const;
    [[nodiscard]] double clamp(double value) const;
    [[nodiscard]] bool contains(double value) const;

    [[nodiscard]] static Interval hull(const Interval &a, const Interval &b);
    [[nodiscard]] static Interval hull(const Interval &interval, double value);
    [[nodiscard]] static Interval hull(double a, double b);

    template<RangeOf<Interval> IntervalRange>
    [[nodiscard]] static Interval hull(const IntervalRange &intervals);

 private:
    double _low;
    double _high;
};

inline Interval::Interval() :
    _low(std::numeric_limits<double>::max()),
    _high(std::numeric_limits<double>::lowest()) {
};

inline Interval::Interval(double low, double high) :
    _low(low),
    _high(high) {
};

template<RangeOf<double> DoubleRange>
Interval::Interval(const DoubleRange &range) {
    auto [min_it, max_it] = std::ranges::minmax_element(range);

    _low = *min_it;
    _high = *max_it;
}

inline double Interval::low() const {
    return _low;
}

inline double Interval::high() const {
    return _high;
}

inline double Interval::measure() const {
    return _high - _low;
}

inline double Interval::midpoint() const {
    return (_low + _high) / 2.0;
}

inline double Interval::clamp(double value) const {
    return std::clamp(value, _low, _high);
}

inline bool Interval::contains(double value) const {
    return _low <= value && value <= _high;
}

inline Interval Interval::hull(const Interval &a, const Interval &b) {
    return Interval(
        std::min(a._low, b._low),
        std::max(a._high, b._high)
    );
}

inline Interval Interval::hull(const Interval &interval, double value) {
    return Interval(
        std::min(interval._low, value),
        std::max(interval._high, value)
    );
}

inline Interval Interval::hull(double a, double b) {
    return Interval(
        std::min(a, b),
        std::max(a, b)
    );
}

template<RangeOf<Interval> IntervalRange>
Interval Interval::hull(const IntervalRange &intervals) {
    CGAL_precondition(!std::ranges::empty(intervals));

    return std::accumulate(
        std::next(std::ranges::begin(intervals)),
        std::ranges::end(intervals),
        *std::ranges::begin(intervals),
        [](const Interval &a, const Interval &b) { return hull(a, b); }
    );
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_RANGE_H_
