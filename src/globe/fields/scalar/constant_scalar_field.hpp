#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_CONSTANT_SCALAR_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_CONSTANT_SCALAR_FIELD_HPP_

#include "../../types.hpp"
#include "../../math/interval.hpp"

namespace globe {

class ConstantScalarField {
public:
    explicit ConstantScalarField(double value = 1.0, Interval output_range = Interval(0, 1)) :
        _value(value),
        _output_range(output_range) {}

    inline double value(const Point3 &) const {
        return _value;
    }

    inline Interval output_range() const {
        return _output_range;
    }

private:
    double _value;
    Interval _output_range;
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_CONSTANT_SCALAR_FIELD_HPP_

