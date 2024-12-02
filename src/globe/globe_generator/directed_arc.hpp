#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_DIRECTED_ARC_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_DIRECTED_ARC_HPP_

#include "../points_collection/types.hpp"
#include "../types.hpp"
#include <CGAL/Cartesian_converter.h>
#include <utility>

namespace globe {

class DirectedArc {
 public:
    explicit DirectedArc(Point3 p, Point3 q);

 private:
    Arc _arc;
    bool _reverse;

    static bool is_canonicalized(const SphericalVector3 &vector);
};

DirectedArc::DirectedArc(Point3 p, Point3 q) {
    _arc = Traits().construct_arc_on_sphere_2_object()(p, q);
    _reverse = is_canonicalized(_arc.supporting_plane().orthogonal_vector());
}

bool DirectedArc::is_canonicalized(const SphericalVector3 &vector) {
    if (vector.x() != 0) {
        return vector.x() > 0;
    } else if (vector.y() != 0) {
        return vector.y() > 0;
    } else {
        return vector.z() > 0;
    }
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_DIRECTED_ARC_HPP_
