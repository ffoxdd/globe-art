#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_RANGE_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_RANGE_H_

#include <limits>
#include <concepts>
#include <ranges>

namespace globe {

template<typename T>
concept DoubleRange = std::ranges::range<T> && std::same_as<std::ranges::range_value_t<T>, double>;

class Interval {
 public:
    Interval();
    Interval(double low, double high);
    template<DoubleRange DoubleRangeType> explicit Interval(const DoubleRangeType &range);

    [[nodiscard]] double low() const;
    [[nodiscard]] double high() const;
    [[nodiscard]] double measure() const;
    [[nodiscard]] double midpoint() const;
    [[nodiscard]] bool contains(double value) const;

 private:
    double _low;
    double _high;

    explicit Interval(const std::pair<double, double> &min_max);

    template<DoubleRange DoubleRangeType> static std::pair<double, double> min_max(const DoubleRangeType &range);
};

inline Interval::Interval() :
    _low(std::numeric_limits<double>::max()),
    _high(std::numeric_limits<double>::lowest()) {
};

inline Interval::Interval(double low, double high) :
    _low(low),
    _high(high) {
};

template<DoubleRange DoubleRangeType>
Interval::Interval(const DoubleRangeType &range): Interval(min_max(range)) {
}

inline Interval::Interval(const std::pair<double, double> &min_max) :
    _low(min_max.first),
    _high(min_max.second) {
}

inline double Interval::low() const {
    return _low;
}

inline double Interval::high() const {
    return _high;
}

template<DoubleRange DoubleRangeType> std::pair<double, double> Interval::min_max(const DoubleRangeType &range) {
    auto [min_it, max_it] = std::ranges::minmax_element(range);
    return {*min_it, *max_it};
}

inline double Interval::measure() const {
    return _high - _low;
}

inline double Interval::midpoint() const {
    return (_low + _high) / 2.0;
}

inline bool Interval::contains(double value) const {
    return _low <= value && value <= _high;
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_RANGE_H_
