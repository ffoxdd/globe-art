#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_CARTESIAN_BOUNDING_BOX_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_CARTESIAN_BOUNDING_BOX_HPP_

#include "../../math/interval.hpp"
#include "../../types.hpp"

namespace globe {

class BoundingBox {
 public:
    BoundingBox();
    BoundingBox(Interval x_interval, Interval y_interval, Interval z_interval);

    [[nodiscard]] Interval x_interval() const;
    [[nodiscard]] Interval y_interval() const;
    [[nodiscard]] Interval z_interval() const;
    [[nodiscard]] Point3 center() const;
    [[nodiscard]] bool contains(const Point3 &point) const;

    [[nodiscard]] static BoundingBox unit_cube();

 private:
    const Interval _x_interval;
    const Interval _y_interval;
    const Interval _z_interval;
};

inline BoundingBox::BoundingBox() :
    BoundingBox(
        Interval(-1, 1),
        Interval(-1, 1),
        Interval(-1, 1)
    ) {
}

inline BoundingBox::BoundingBox(Interval x_interval, Interval y_interval, Interval z_interval) :
    _x_interval(x_interval),
    _y_interval(y_interval),
    _z_interval(z_interval) {
}

inline Interval BoundingBox::x_interval() const {
    return _x_interval;
}

inline Interval BoundingBox::y_interval() const {
    return _y_interval;
}

inline Interval BoundingBox::z_interval() const {
    return _z_interval;
}

inline Point3 BoundingBox::center() const {
    return Point3(
        _x_interval.midpoint(),
        _y_interval.midpoint(),
        _z_interval.midpoint()
    );
}

inline bool BoundingBox::contains(const Point3 &point) const {
    return _x_interval.contains(point.x()) &&
           _y_interval.contains(point.y()) &&
           _z_interval.contains(point.z());
}

inline BoundingBox BoundingBox::unit_cube() {
    return BoundingBox(Interval(-1, 1), Interval(-1, 1), Interval(-1, 1));
}

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_CARTESIAN_BOUNDING_BOX_HPP_
