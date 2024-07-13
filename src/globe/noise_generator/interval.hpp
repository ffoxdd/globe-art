#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_RANGE_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_RANGE_H_

#include <limits>

namespace globe {

class Interval {
 public:
    Interval();
    Interval(double low, double high);

    [[nodiscard]] double low() const;
    [[nodiscard]] double high() const;

    void update_domain(double value);
    static double map(Interval &input_range, Interval &output_range, double value);

 private:
    double _low;
    double _high;

    double t(double value);
    [[nodiscard]] double measure() const;
    double at(double t);
};

Interval::Interval() :
    _low(std::numeric_limits<double>::max()),
    _high(std::numeric_limits<double>::lowest()) {
};

Interval::Interval(double low, double high) :
    _low(low),
    _high(high) {
};

double Interval::low() const {
    return _low;
}

double Interval::high() const {
    return _high;
}

void Interval::update_domain(double value) {
    if (value < _low) {
        _low = value;
    }

    if (value > _high) {
        _high = value;
    }
}

double Interval::map(Interval &input_range, Interval &output_range, double value) {
    return output_range.at(input_range.t(value));
}

double Interval::t(double value) {
    if (measure() == 0) {
        return 0;
    }

    return (value - _low) / measure();
}

double Interval::measure() const {
    return _high - _low;
}

double Interval::at(double t) {
    return _low + (measure() * t);
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_RANGE_H_
