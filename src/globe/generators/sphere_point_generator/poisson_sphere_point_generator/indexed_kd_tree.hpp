#ifndef GLOBEART_SRC_GLOBE_GENERATORS_SPHERE_POINT_GENERATOR_POISSON_SPHERE_POINT_GENERATOR_INDEXED_KD_TREE_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_SPHERE_POINT_GENERATOR_POISSON_SPHERE_POINT_GENERATOR_INDEXED_KD_TREE_HPP_

#include "../../../types.hpp"
#include <CGAL/Search_traits_adapter.h>
#include <boost/property_map/property_map.hpp>
#include <vector>

namespace globe {

template<typename Container>
concept IndexablePointContainer = requires(const Container &container, std::size_t index) {
    { container[index] } -> std::convertible_to<const Point3&>;
};

template<IndexablePointContainer PointContainer>
class IndexedPointMap {
 public:
    using value_type = Point3;
    using reference = const Point3&;
    using key_type = std::size_t;
    using category = boost::readable_property_map_tag;

    explicit IndexedPointMap(const PointContainer &points) : _points(points) {}

    reference operator[](key_type index) const {
        return _points[index];
    }

    friend reference get(const IndexedPointMap &map, key_type index) {
        return map._points[index];
    }

 private:
    const PointContainer &_points;
};

using IndexedSearchTraits = CGAL::Search_traits_adapter<
    std::size_t,
    IndexedPointMap<std::vector<Point3>>,
    SearchTraits
>;

using IndexedKDTree = CGAL::Kd_tree<IndexedSearchTraits>;
using IndexedFuzzySphere = CGAL::Fuzzy_sphere<IndexedSearchTraits>;

}

#endif //GLOBEART_SRC_GLOBE_GENERATORS_SPHERE_POINT_GENERATOR_POISSON_SPHERE_POINT_GENERATOR_INDEXED_KD_TREE_HPP_
