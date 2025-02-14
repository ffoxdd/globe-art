#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_DIRECTED_ARC_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_DIRECTED_ARC_HPP_

#include "../points_collection/types.hpp"
#include "../types.hpp"

namespace globe {
class DirectedArc {
public:
    explicit DirectedArc(CircularArc3 arc);
    [[nodiscard]] double approximate_angle() const;

private:
    Arc _arc;

    static bool is_canonicalized(const CircularArc3 &arc);
    static bool is_canonicalized(const SphericalPlane3 &plane);
    static bool is_canonicalized(const SphericalVector3 &vector);
    static SphericalVector3 arc_normal(const CircularArc3 &arc);
    static Arc dual_arc(const CircularArc3 &arc);
};

inline DirectedArc::DirectedArc(CircularArc3 arc) {
    if (!is_canonicalized(arc)) {
        _arc = dual_arc(arc);
    } else {
        _arc = std::move(arc);
    }
}

inline double DirectedArc::approximate_angle() const {
    return _arc.approximate_angle();
}

inline bool DirectedArc::is_canonicalized(const CircularArc3 &arc) {
    return is_canonicalized(arc.supporting_plane());
}

inline bool DirectedArc::is_canonicalized(const SphericalPlane3 &plane) {
    return is_canonicalized(plane.orthogonal_direction().vector());
}

inline bool DirectedArc::is_canonicalized(const SphericalVector3 &vector) {
    if (vector.x() != 0) {
        return vector.x() > 0;
    }

    if (vector.y() != 0) {
        return vector.y() > 0;
    }

    return vector.z() > 0;
}

inline Arc DirectedArc::dual_arc(const CircularArc3 &arc) {
    const auto &circle = arc.supporting_circle();

    SphericalCircle3 opposite_circle(
        circle.center(),
        circle.squared_radius(),
        circle.supporting_plane().opposite()
    );

    return {opposite_circle, arc.target(), arc.source()};
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_DIRECTED_ARC_HPP_
