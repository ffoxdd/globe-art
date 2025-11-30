#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_CONSTANT_SCALAR_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_CONSTANT_SCALAR_FIELD_HPP_

#include "../../types.hpp"

namespace globe {

class ConstantScalarField {
public:
    explicit ConstantScalarField(double value = 1.0) :
        _value(value) {}

    inline double value(const Point3 &) const {
        return _value;
    }

    inline double max_frequency() const {
        return 0.0;
    }

private:
    double _value;
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_CONSTANT_SCALAR_FIELD_HPP_
