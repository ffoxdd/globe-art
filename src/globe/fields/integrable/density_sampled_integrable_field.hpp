#ifndef GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_DENSITY_SAMPLED_INTEGRABLE_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_DENSITY_SAMPLED_INTEGRABLE_FIELD_HPP_

#include "../scalar/scalar_field.hpp"
#include "../../math/interval.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../geometry/spherical/spherical_bounding_box.hpp"
#include "../../generators/sphere_point_generator/sphere_point_generator.hpp"
#include "../../math/interval_sampler/interval_sampler.hpp"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_sphere.h>
#include <vector>
#include <iterator>
#include <algorithm>

namespace globe {

template<
    ScalarField ScalarFieldType,
    SpherePointGenerator GeneratorType,
    IntervalSampler IntervalSamplerType = UniformIntervalSampler
>
class DensitySampledIntegrableField {
 public:
    DensitySampledIntegrableField(
        ScalarFieldType &scalar_field,
        GeneratorType point_generator,
        size_t target_sample_count = 200'000,
        double max_density = 1.0,
        IntervalSamplerType interval_sampler = IntervalSamplerType()
    );

    [[nodiscard]] double integrate(const SphericalPolygon &polygon) const;
    [[nodiscard]] double integrate(const SphericalBoundingBox &bounding_box = SphericalBoundingBox::full_sphere()) const;

 private:
    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using SearchTraits = CGAL::Search_traits_3<Kernel>;
    using KdTree = CGAL::Kd_tree<SearchTraits>;
    using FuzzySphere = CGAL::Fuzzy_sphere<SearchTraits>;

    ScalarFieldType &_scalar_field;
    GeneratorType _point_generator;
    SphericalBoundingBox _global_bounding_box;
    std::vector<Point3> _points;
    KdTree _kdtree;
    double _weight_per_sample;
    double _max_density;
    size_t _sample_attempts;
    IntervalSamplerType _interval_sampler;

    void build_samples(size_t target_sample_count);
    void try_add_sample();
    [[nodiscard]] Point3 generate_candidate();
    [[nodiscard]] bool candidate_accepted(const Point3 &candidate);
    [[nodiscard]] double acceptance_threshold(const Point3 &point);
    [[nodiscard]] double sample_acceptance();

    void build_kdtree();
    [[nodiscard]] double weight_per_sample(size_t attempts) const;
    [[nodiscard]] FuzzySphere query_sphere(const SphericalPolygon &polygon) const;
    [[nodiscard]] std::vector<Point3> candidate_points(const SphericalPolygon &polygon) const;
    [[nodiscard]] size_t contained_points_count(const SphericalPolygon &polygon) const;
};

template<ScalarField ScalarFieldType, SpherePointGenerator GeneratorType, IntervalSampler IntervalSamplerType>
DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::DensitySampledIntegrableField(
    ScalarFieldType &scalar_field,
    GeneratorType point_generator,
    size_t target_sample_count,
    double max_density,
    IntervalSamplerType interval_sampler
) :
    _scalar_field(scalar_field),
    _point_generator(std::move(point_generator)),
    _global_bounding_box(SphericalBoundingBox::full_sphere()),
    _weight_per_sample(1.0),
    _max_density(std::max(max_density, 1e-6)),
    _sample_attempts(0),
    _interval_sampler(std::move(interval_sampler)) {

    build_samples(target_sample_count);
}

template<ScalarField ScalarFieldType, SpherePointGenerator GeneratorType, IntervalSampler IntervalSamplerType>
void DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::build_samples(
    size_t target_sample_count
) {
    if (target_sample_count == 0) {
        return;
    }

    size_t attempts = 0;

    while (_points.size() < target_sample_count) {
        attempts++;
        try_add_sample();
    }

    _sample_attempts = attempts;
    _weight_per_sample = weight_per_sample(attempts);

    build_kdtree();
}

template<ScalarField ScalarFieldType, SpherePointGenerator GeneratorType, IntervalSampler IntervalSamplerType>
void DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::try_add_sample() {
    Point3 candidate = generate_candidate();

    if (!candidate_accepted(candidate)) {
        return;
    }

    _points.push_back(candidate);
}

template<ScalarField ScalarFieldType, SpherePointGenerator GeneratorType, IntervalSampler IntervalSamplerType>
Point3 DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::generate_candidate() {
    return _point_generator.generate(_global_bounding_box);
}

template<ScalarField ScalarFieldType, SpherePointGenerator GeneratorType, IntervalSampler IntervalSamplerType>
bool DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::candidate_accepted(const Point3 &candidate) {
    return sample_acceptance() <= acceptance_threshold(candidate);
}

template<ScalarField ScalarFieldType, SpherePointGenerator GeneratorType, IntervalSampler IntervalSamplerType>
double DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::acceptance_threshold(const Point3 &point) {
    double density = _scalar_field.value(point);
    CGAL_precondition(density >= 0.0 && density <= 1.0);
    return std::clamp(density / _max_density, 0.0, 1.0);
}

template<ScalarField ScalarFieldType, SpherePointGenerator GeneratorType, IntervalSampler IntervalSamplerType>
double DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::sample_acceptance() {
    return _interval_sampler.sample(Interval(0.0, 1.0));
}

template<ScalarField ScalarFieldType, SpherePointGenerator GeneratorType, IntervalSampler IntervalSamplerType>
double DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::integrate(
    const SphericalPolygon &polygon
) const {
    if (_points.empty()) {
        return 0.0;
    }

    size_t hits = contained_points_count(polygon);
    return static_cast<double>(hits) * _weight_per_sample;
}

template<ScalarField ScalarFieldType, SpherePointGenerator GeneratorType, IntervalSampler IntervalSamplerType>
void DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::build_kdtree() {
    _kdtree.insert(_points.begin(), _points.end());
    _kdtree.build();
}

template<ScalarField ScalarFieldType, SpherePointGenerator GeneratorType, IntervalSampler IntervalSamplerType>
double DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::weight_per_sample(size_t attempts) const {
    return (4.0 * M_PI * _max_density) / static_cast<double>(attempts);
}

template<ScalarField ScalarFieldType, SpherePointGenerator GeneratorType, IntervalSampler IntervalSamplerType>
typename DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::FuzzySphere DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::query_sphere(
    const SphericalPolygon &polygon
) const {
    Point3 centroid = polygon.centroid();
    double max_squared_distance = 0.0;

    for (const auto& v : polygon.points()) {
        double squared_dist = CGAL::squared_distance(v, centroid);
        max_squared_distance = std::max(max_squared_distance, squared_dist);
    }

    double max_chord_distance = std::sqrt(max_squared_distance);
    return FuzzySphere(centroid, max_chord_distance, 1e-9);
}

template<ScalarField ScalarFieldType, SpherePointGenerator GeneratorType, IntervalSampler IntervalSamplerType>
std::vector<Point3> DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::candidate_points(
    const SphericalPolygon &polygon
) const {
    FuzzySphere query = query_sphere(polygon);
    std::vector<Point3> candidates;
    _kdtree.search(std::back_inserter(candidates), query);

    return candidates;
}

template<ScalarField ScalarFieldType, SpherePointGenerator GeneratorType, IntervalSampler IntervalSamplerType>
size_t DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::contained_points_count(
    const SphericalPolygon &polygon
) const {
    auto candidates = candidate_points(polygon);

    return std::count_if(
        candidates.begin(),
        candidates.end(),
        [&polygon](const Point3 &point) { return polygon.contains(point); }
    );
}

template<ScalarField ScalarFieldType, SpherePointGenerator GeneratorType, IntervalSampler IntervalSamplerType>
double DensitySampledIntegrableField<ScalarFieldType, GeneratorType, IntervalSamplerType>::integrate(const SphericalBoundingBox &bounding_box) const {
    if (_points.empty()) {
        return 0.0;
    }

    Point3 center = bounding_box.center();
    double radius = bounding_box.bounding_sphere_radius();
    FuzzySphere query(center, radius, 1e-9);

    std::vector<Point3> candidates;
    _kdtree.search(std::back_inserter(candidates), query);

    size_t hits = std::count_if(
        candidates.begin(),
        candidates.end(),
        [&bounding_box](const Point3 &point) { return bounding_box.contains(point); }
    );

    return static_cast<double>(hits) * _weight_per_sample;
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_DENSITY_SAMPLED_INTEGRABLE_FIELD_HPP_

