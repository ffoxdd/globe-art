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
    template<DoubleRange DR> explicit Interval(const DR &range);

    [[nodiscard]] double low() const;
    [[nodiscard]] double high() const;
    [[nodiscard]] double measure() const;
    [[nodiscard]] double midpoint() const;
    [[nodiscard]] Interval lower_half() const;
    [[nodiscard]] Interval upper_half() const;
    [[nodiscard]] bool overlaps(const Interval &other) const;
    [[nodiscard]] bool contains(double value) const;

    static double map(Interval &input_range, Interval &output_range, double value);

 private:
    double _low;
    double _high;

    explicit Interval(const std::pair<double, double> &min_max);

    [[nodiscard]] double t(double value) const;
    [[nodiscard]] double at(double t) const;

    template<DoubleRange DR> static std::pair<double, double> min_max(const DR &range);
};

inline Interval::Interval() :
    _low(std::numeric_limits<double>::max()),
    _high(std::numeric_limits<double>::lowest()) {
};

inline Interval::Interval(double low, double high) :
    _low(low),
    _high(high) {
};

template<DoubleRange DR>
Interval::Interval(const DR &range): Interval(min_max(range)) {
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

inline double Interval::map(Interval &input_range, Interval &output_range, double value) {
    return output_range.at(input_range.t(value));
}

template<DoubleRange DR> std::pair<double, double> Interval::min_max(const DR &range) {
    auto [min_it, max_it] = std::ranges::minmax_element(range);
    return {*min_it, *max_it};
}

inline double Interval::t(double value) const {
    if (measure() == 0) {
        return 0;
    }

    return (value - _low) / measure();
}

inline double Interval::measure() const {
    return _high - _low;
}

inline double Interval::at(double t) const {
    return _low + (measure() * t);
}

inline double Interval::midpoint() const {
    return (_low + _high) / 2.0;
}

inline Interval Interval::lower_half() const {
    return Interval(_low, midpoint());
}

inline Interval Interval::upper_half() const {
    return Interval(midpoint(), _high);
}

inline bool Interval::overlaps(const Interval &other) const {
    return !(_high < other._low || _low > other._high);
}

inline bool Interval::contains(double value) const {
    return _low <= value && value <= _high;
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_RANGE_H_
