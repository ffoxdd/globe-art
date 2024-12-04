#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_DIRECTED_ARC_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_DIRECTED_ARC_HPP_

#include "../points_collection/types.hpp"
#include "../types.hpp"
#include <CGAL/Cartesian_converter.h>
#include <utility>

namespace globe {

class DirectedArc {
 public:
    DirectedArc(CircularArc3 arc, bool reverse);
    double approximate_angle() const;

 private:
    Arc _arc;
    bool _reverse;
    // static bool is_canonicalized(const SphericalVector3 &vector);
};

DirectedArc::DirectedArc(CircularArc3 arc, bool reverse) :
    _arc(std::move(arc)),
    _reverse(reverse) {
}

double DirectedArc::approximate_angle() const {
    if (_reverse) {
        return 2 * M_PI - _arc.approximate_angle();
    } else {
        return _arc.approximate_angle();
    }
}

//bool DirectedArc::is_canonicalized(const SphericalVector3 &vector) {
//    if (vector.x() != 0) {
//        return vector.x() > 0;
//    } else if (vector.y() != 0) {
//        return vector.y() > 0;
//    } else {
//        return vector.z() > 0;
//    }
//}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_DIRECTED_ARC_HPP_
