#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_RANGE_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_RANGE_H_

#include <limits>

namespace globe {

class Range {
 public:
    Range() :
        _low(std::numeric_limits<double>::max()),
        _high(std::numeric_limits<double>::lowest()) { };

    Range(double low, double high) : _low(low), _high(high) { };

    void update_domain(double value);
    static double map(Range &input_range, Range &output_range, double value);

 private:
    double _low;
    double _high;

    double t(double value);
    [[nodiscard]] double measure() const;
    double at(double t);
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_RANGE_H_
