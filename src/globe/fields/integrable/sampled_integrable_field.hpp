#ifndef GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_SAMPLED_INTEGRABLE_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_SAMPLED_INTEGRABLE_FIELD_HPP_

#include "../../math/interval.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../geometry/spherical/spherical_bounding_box.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "../../generators/sphere_point_generator/sphere_point_generator.hpp"
#include "../../generators/sphere_point_generator/rejection_sampling_sphere_point_generator.hpp"
#include <vector>
#include <iterator>
#include <algorithm>

namespace globe {

template<SpherePointGenerator GeneratorType = RejectionSamplingSpherePointGenerator<>>
class SampledIntegrableField {
 public:
    SampledIntegrableField(
        GeneratorType point_generator,
        size_t target_sample_count = 200'000,
        double max_density = 1.0
    );

    [[nodiscard]] double integrate(const SphericalPolygon &polygon) const;
    [[nodiscard]] double integrate(const SphericalBoundingBox &bounding_box = SphericalBoundingBox::full_sphere()) const;
    [[nodiscard]] double max_frequency() const;

 private:
    GeneratorType _point_generator;
    SphericalBoundingBox _global_bounding_box;
    std::vector<Point3> _points;
    KDTree _kdtree;
    double _weight_per_sample;
    double _max_density;
    size_t _sample_attempts;

    void build_samples(size_t target_sample_count);
    void build_kdtree();
    [[nodiscard]] double weight_per_sample(size_t attempts) const;
    [[nodiscard]] FuzzySphere query_sphere(const SphericalPolygon &polygon) const;
    [[nodiscard]] std::vector<Point3> candidate_points(const SphericalPolygon &polygon) const;
    [[nodiscard]] size_t contained_points_count(const SphericalPolygon &polygon) const;
};

template<SpherePointGenerator GeneratorType>
SampledIntegrableField<GeneratorType>::SampledIntegrableField(
    GeneratorType point_generator,
    size_t target_sample_count,
    double max_density
) :
    _point_generator(std::move(point_generator)),
    _global_bounding_box(SphericalBoundingBox::full_sphere()),
    _weight_per_sample(1.0),
    _max_density(std::max(max_density, 1e-6)),
    _sample_attempts(0) {
    build_samples(target_sample_count);
}

template<SpherePointGenerator GeneratorType>
void SampledIntegrableField<GeneratorType>::build_samples(
    size_t target_sample_count
) {
    if (target_sample_count == 0) {
        return;
    }

    _points = _point_generator.generate(target_sample_count, _global_bounding_box);
    _sample_attempts = _point_generator.last_attempt_count();
    _weight_per_sample = weight_per_sample(_sample_attempts);

    build_kdtree();
}

template<SpherePointGenerator GeneratorType>
double SampledIntegrableField<GeneratorType>::integrate(
    const SphericalPolygon &polygon
) const {
    if (_points.empty()) {
        return 0.0;
    }

    size_t hits = contained_points_count(polygon);
    return static_cast<double>(hits) * _weight_per_sample;
}

template<SpherePointGenerator GeneratorType>
void SampledIntegrableField<GeneratorType>::build_kdtree() {
    _kdtree.insert(_points.begin(), _points.end());
    _kdtree.build();
}

template<SpherePointGenerator GeneratorType>
double SampledIntegrableField<GeneratorType>::weight_per_sample(size_t attempts) const {
    return (4.0 * M_PI * _max_density) / static_cast<double>(attempts);
}

template<SpherePointGenerator GeneratorType>
FuzzySphere SampledIntegrableField<GeneratorType>::query_sphere(
    const SphericalPolygon &polygon
) const {
    return FuzzySphere(polygon.centroid(), polygon.bounding_sphere_radius(), GEOMETRIC_EPSILON);
}

template<SpherePointGenerator GeneratorType>
std::vector<Point3> SampledIntegrableField<GeneratorType>::candidate_points(
    const SphericalPolygon &polygon
) const {
    FuzzySphere query = query_sphere(polygon);
    std::vector<Point3> candidates;
    _kdtree.search(std::back_inserter(candidates), query);

    return candidates;
}

template<SpherePointGenerator GeneratorType>
size_t SampledIntegrableField<GeneratorType>::contained_points_count(
    const SphericalPolygon &polygon
) const {
    auto candidates = candidate_points(polygon);

    return std::count_if(
        candidates.begin(),
        candidates.end(),
        [&polygon](const Point3 &point) { return polygon.contains(point); }
    );
}

template<SpherePointGenerator GeneratorType>
double SampledIntegrableField<GeneratorType>::integrate(const SphericalBoundingBox &bounding_box) const {
    if (_points.empty()) {
        return 0.0;
    }

    Point3 center = bounding_box.center();
    double radius = bounding_box.bounding_sphere_radius();
    FuzzySphere query(center, radius, GEOMETRIC_EPSILON);

    std::vector<Point3> candidates;
    _kdtree.search(std::back_inserter(candidates), query);

    size_t hits = std::count_if(
        candidates.begin(),
        candidates.end(),
        [&bounding_box](const Point3 &point) { return bounding_box.contains(point); }
    );

    return static_cast<double>(hits) * _weight_per_sample;
}

template<SpherePointGenerator GeneratorType>
double SampledIntegrableField<GeneratorType>::max_frequency() const {
    if (_points.empty()) {
        return 0.0;
    }
    double average_spacing = std::sqrt(UNIT_SPHERE_AREA / static_cast<double>(_points.size()));
    return 1.0 / (2.0 * average_spacing);
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_SAMPLED_INTEGRABLE_FIELD_HPP_

