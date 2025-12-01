#ifndef GLOBEART_SRC_GLOBE_FIELDS_SCALAR_CONSTANT_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SCALAR_CONSTANT_FIELD_HPP_

#include "../../types.hpp"

namespace globe::fields::scalar {

class ConstantField {
public:
    explicit ConstantField(double value = 1.0) :
        _value(value) {}

    inline double value(const VectorS2 &) const {
        return _value;
    }

    inline double max_frequency() const {
        return 0.0;
    }

private:
    double _value;
};

} // namespace globe::fields::scalar

#endif //GLOBEART_SRC_GLOBE_FIELDS_SCALAR_CONSTANT_FIELD_HPP_
