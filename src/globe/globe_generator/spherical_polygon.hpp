#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_POLYGON_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_POLYGON_HPP_

#include "../points_collection/types.hpp" // TODO: consider moving this closer to points_collection
#include <utility>

namespace globe {

class SphericalBoundingBox {
};


class SphericalPolygon {
 public:
    explicit SphericalPolygon(std::vector<Arc> arcs);
    [[nodiscard]] SphericalBoundingBox bounding_box() const;

 private:
    std::vector<Arc> _arcs;
};

SphericalPolygon::SphericalPolygon(std::vector<Arc> arcs) :
    _arcs(std::move(arcs)) {
}

SphericalBoundingBox SphericalPolygon::bounding_box() const {
    return SphericalBoundingBox();
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SPHERICAL_POLYGON_HPP_
