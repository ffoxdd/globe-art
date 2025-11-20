#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_CONSTANT_SCALAR_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_CONSTANT_SCALAR_FIELD_HPP_

#include "../types.hpp"
#include "interval.hpp"
#include <vector>

namespace globe {

class ConstantScalarField {
public:
    explicit ConstantScalarField(double value = 1.0) : _value(value) {}

    inline void normalize(const std::vector<Point3> &, Interval output_interval = Interval(0, 1)) {
        const double midpoint = (output_interval.low() + output_interval.high()) / 2.0;
        _value = midpoint;
    }

    inline double value(const Point3 &) const {
        return _value;
    }

private:
    double _value;
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_CONSTANT_SCALAR_FIELD_HPP_

