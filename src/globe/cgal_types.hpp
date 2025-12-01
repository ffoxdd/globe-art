#ifndef GLOBEART_SRC_GLOBE_CGAL_TYPES_HPP_
#define GLOBEART_SRC_GLOBE_CGAL_TYPES_HPP_

#include "types.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

namespace globe {

template<typename T>
concept HasXYZ = requires(const T &value) { value.x(); value.y(); value.z(); };

namespace detail {
    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
}

using Vector3 = detail::Kernel::Vector_3;
using Point3 = detail::Kernel::Point_3;

inline Point3 to_cgal_point(const VectorS2& v) {
    return Point3(v.x(), v.y(), v.z());
}

template<HasXYZ PointType>
inline Vector3 to_cgal_vector(const PointType &point) {
    return Vector3(
        CGAL::to_double(point.x()),
        CGAL::to_double(point.y()),
        CGAL::to_double(point.z())
    );
}

inline VectorS2 to_vector_s2(const Point3& p) {
    return VectorS2(p.x(), p.y(), p.z());
}

inline VectorS2 to_vector_s2(const Vector3& v) {
    return VectorS2(v.x(), v.y(), v.z());
}

}

#endif //GLOBEART_SRC_GLOBE_CGAL_TYPES_HPP_
