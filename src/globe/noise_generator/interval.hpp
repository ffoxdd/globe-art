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

    static double map(Interval &input_range, Interval &output_range, double value);

 private:
    double _low;
    double _high;

    explicit Interval(const std::pair<double, double> &min_max);

    [[nodiscard]] double t(double value) const;
    [[nodiscard]] double at(double t) const;

    template<DoubleRange DR> static std::pair<double, double> min_max(const DR &range);
};

Interval::Interval() :
    _low(std::numeric_limits<double>::max()),
    _high(std::numeric_limits<double>::lowest()) {
};

Interval::Interval(double low, double high) :
    _low(low),
    _high(high) {
};

template<DoubleRange DR>
Interval::Interval(const DR &range): Interval(min_max(range)) {
}

Interval::Interval(const std::pair<double, double> &min_max) :
    _low(min_max.first),
    _high(min_max.second) {
}

double Interval::low() const {
    return _low;
}

double Interval::high() const {
    return _high;
}

double Interval::map(Interval &input_range, Interval &output_range, double value) {
    return output_range.at(input_range.t(value));
}

template<DoubleRange DR> std::pair<double, double> Interval::min_max(const DR &range) {
    auto [min_it, max_it] = std::ranges::minmax_element(range);
    return {*min_it, *max_it};
}

double Interval::t(double value) const {
    if (measure() == 0) {
        return 0;
    }

    return (value - _low) / measure();
}

double Interval::measure() const {
    return _high - _low;
}

double Interval::at(double t) const {
    return _low + (measure() * t);
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_RANGE_H_
