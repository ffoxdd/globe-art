#include "range.h"
#include <iostream>

namespace globe {

void Range::update_domain(double value) {
    if (value < _low) {
        _low = value;
    }

    if (value > _high) {
        _high = value;
    }
}

double Range::map(Range &input_range, Range &output_range, double value) {
    return output_range.at(input_range.t(value));
}

double Range::t(double value) {
    if (measure() == 0) {
        return 0;
    }

    return (value - _low) / measure();
}

double Range::measure() const {
    return _high - _low;
}

double Range::at(double t) {
    return _low + (measure() * t);
}

} // namespace globe