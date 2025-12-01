#ifndef GLOBEART_SRC_GLOBE_FIELDS_SCALAR_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SCALAR_FIELD_HPP_

#include "../../types.hpp"

namespace globe::fields::scalar {

template<typename T>
concept Field = requires( T field, const VectorS2 &point ) {
    { field.value(point) } -> std::convertible_to<double>;
};

} // namespace globe::fields::scalar

#endif //GLOBEART_SRC_GLOBE_FIELDS_SCALAR_FIELD_HPP_
