#ifndef GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_DENSITY_SAMPLED_INTEGRABLE_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_DENSITY_SAMPLED_INTEGRABLE_FIELD_HPP_

#include "../scalar_field/scalar_field.hpp"
#include "../scalar_field/interval.hpp"
#include "../globe_generator/spherical_polygon.hpp"
#include "../globe_generator/spherical_bounding_box.hpp"
#include "../globe_generator/sample_point_generator/bounding_box_sample_point_generator.hpp"
#include "../globe_generator/interval_sampler.hpp"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_sphere.h>
#include <vector>
#include <cmath>
#include <iterator>
#include <algorithm>

namespace globe {

template<ScalarField ScalarFieldType>
class DensitySampledIntegrableField {
 public:
    DensitySampledIntegrableField(
        ScalarFieldType &scalar_field,
        size_t target_sample_count = 200'000,
        double max_density = 1.0
    );

    [[nodiscard]] double integrate(const SphericalPolygon &polygon) const;
    [[nodiscard]] double integrate_entire_sphere() const;
    [[nodiscard]] size_t sample_count() const;

 private:
    using SearchTraits = CGAL::Search_traits_3<Kernel>;
    using KdTree = CGAL::Kd_tree<SearchTraits>;
    using FuzzySphere = CGAL::Fuzzy_sphere<SearchTraits>;

    ScalarFieldType &_scalar_field;
    BoundingBoxSamplePointGenerator _point_generator;
    SphericalBoundingBox _global_bounding_box;
    std::vector<Point3> _points;
    KdTree _kdtree;
    double _weight_per_sample;
    double _max_density;
    size_t _sample_attempts;
    IntervalSampler _interval_sampler;

    void build_samples(size_t target_sample_count);
    [[nodiscard]] double search_radius(const SphericalPolygon &polygon) const;
    [[nodiscard]] double acceptance_threshold(const Point3 &point) const;
    [[nodiscard]] double sample_acceptance();
    [[nodiscard]] bool should_accept(const Point3 &candidate);
    [[nodiscard]] Point3 generate_candidate();
    void try_add_sample();
    void build_kdtree();
    [[nodiscard]] double weight_per_sample(size_t attempts) const;
};

template<ScalarField ScalarFieldType>
DensitySampledIntegrableField<ScalarFieldType>::DensitySampledIntegrableField(
    ScalarFieldType &scalar_field,
    size_t target_sample_count,
    double max_density
) :
    _scalar_field(scalar_field),
    _point_generator(),
    _global_bounding_box(),
    _weight_per_sample(1.0),
    _max_density(std::max(max_density, 1e-6)),
    _sample_attempts(0) {
    build_samples(target_sample_count);
}

template<ScalarField ScalarFieldType>
void DensitySampledIntegrableField<ScalarFieldType>::build_samples(
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

template<ScalarField ScalarFieldType>
double DensitySampledIntegrableField<ScalarFieldType>::integrate(
    const SphericalPolygon &polygon
) const {
    if (_points.empty()) {
        return 0.0;
    }

    double radius = search_radius(polygon);

    auto bounding_box = polygon.bounding_box();
    Point3 center = bounding_box.center();
    FuzzySphere query(center, radius, 0.0);
    std::vector<Point3> candidates;
    _kdtree.search(std::back_inserter(candidates), query);

    size_t hits = 0;
    for (const auto &point : candidates) {
        if (polygon.contains(point)) {
            hits++;
        }
    }

    return static_cast<double>(hits) * _weight_per_sample;
}

template<ScalarField ScalarFieldType>
double DensitySampledIntegrableField<ScalarFieldType>::search_radius(
    const SphericalPolygon &polygon
) const {
    auto bbox = polygon.bounding_box();
    auto z_interval = bbox.z_interval();
    auto theta_interval = bbox.theta_interval();

    double z_span = z_interval.measure();
    double r_low = std::sqrt(std::max(0.0, 1.0 - z_interval.low() * z_interval.low()));
    double r_high = std::sqrt(std::max(0.0, 1.0 - z_interval.high() * z_interval.high()));
    double r_max = std::max(r_low, r_high);
    double chord = 2.0 * r_max * std::sin(theta_interval.measure() / 2.0);
    double radius = std::sqrt(z_span * z_span + chord * chord) * 1.1;

    return std::max(radius, 0.05);
}

template<ScalarField ScalarFieldType>
double DensitySampledIntegrableField<ScalarFieldType>::acceptance_threshold(const Point3 &point) const {
    double density = _scalar_field.value(point);
    return std::clamp(density / _max_density, 0.0, 1.0);
}

template<ScalarField ScalarFieldType>
double DensitySampledIntegrableField<ScalarFieldType>::sample_acceptance() {
    return _interval_sampler.sample(Interval(0.0, 1.0));
}

template<ScalarField ScalarFieldType>
bool DensitySampledIntegrableField<ScalarFieldType>::should_accept(const Point3 &candidate) {
    return sample_acceptance() <= acceptance_threshold(candidate);
}

template<ScalarField ScalarFieldType>
Point3 DensitySampledIntegrableField<ScalarFieldType>::generate_candidate() {
    return _point_generator.generate(_global_bounding_box);
}

template<ScalarField ScalarFieldType>
void DensitySampledIntegrableField<ScalarFieldType>::try_add_sample() {
    Point3 candidate = generate_candidate();

    if (should_accept(candidate)) {
        _points.push_back(candidate);
    }
}

template<ScalarField ScalarFieldType>
void DensitySampledIntegrableField<ScalarFieldType>::build_kdtree() {
    _kdtree.insert(_points.begin(), _points.end());
    _kdtree.build();
}

template<ScalarField ScalarFieldType>
double DensitySampledIntegrableField<ScalarFieldType>::weight_per_sample(size_t attempts) const {
    return (4.0 * M_PI * _max_density) / static_cast<double>(attempts);
}

template<ScalarField ScalarFieldType>
double DensitySampledIntegrableField<ScalarFieldType>::integrate_entire_sphere() const {
    return static_cast<double>(_points.size()) * _weight_per_sample;
}

template<ScalarField ScalarFieldType>
size_t DensitySampledIntegrableField<ScalarFieldType>::sample_count() const {
    return _points.size();
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_DENSITY_SAMPLED_INTEGRABLE_FIELD_HPP_

