#ifndef GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SAMPLED_SPHERICAL_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SAMPLED_SPHERICAL_FIELD_HPP_

#include "spherical_field.hpp"
#include "../../cgal_types.hpp"
#include "../../geometry/spherical/spherical_arc.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../geometry/spherical/spherical_bounding_box.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "../../generators/sphere_point_generator/sphere_point_generator.hpp"
#include "../../generators/sphere_point_generator/random_sphere_point_generator.hpp"
#include "../scalar/scalar_field.hpp"
#include "../scalar/noise_field.hpp"
#include <CGAL/Search_traits_3.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_sphere.h>
#include <Eigen/Core>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

namespace globe {

template<
    ScalarField ScalarFieldType,
    SpherePointGenerator SpherePointGeneratorType = RandomSpherePointGenerator<>
>
class SampledSphericalField {
 public:
    SampledSphericalField(
        ScalarFieldType field,
        SpherePointGeneratorType point_generator = SpherePointGeneratorType(),
        size_t sample_count = 200'000,
        size_t edge_samples = 50
    );

    SampledSphericalField(const SampledSphericalField&) = delete;
    SampledSphericalField& operator=(const SampledSphericalField&) = delete;
    SampledSphericalField(SampledSphericalField&&) = default;
    SampledSphericalField& operator=(SampledSphericalField&&) = default;

    [[nodiscard]] double value(const VectorS2& point) const;
    [[nodiscard]] double mass(const SphericalPolygon& polygon) const;
    [[nodiscard]] double total_mass() const;
    [[nodiscard]] double edge_integral(const SphericalArc& arc) const;
    [[nodiscard]] Eigen::Vector3d edge_gradient_integral(const SphericalArc& arc) const;

 private:
    using SearchTraits = CGAL::Search_traits_3<detail::Kernel>;
    using KDTree = CGAL::Kd_tree<SearchTraits>;
    using FuzzySphere = CGAL::Fuzzy_sphere<SearchTraits>;

    struct Sample {
        Point3 point;
        double value;
    };

    ScalarFieldType _field;
    std::vector<Sample> _samples;
    std::unique_ptr<KDTree> _kdtree;
    double _weight_per_sample;
    double _total_mass;
    size_t _edge_samples;

    void build_samples(
        SpherePointGeneratorType& point_generator,
        size_t sample_count
    );
    void build_kdtree();

    [[nodiscard]] std::vector<size_t> find_samples_in_polygon(
        const SphericalPolygon& polygon
    ) const;
};

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType>
SampledSphericalField<ScalarFieldType, SpherePointGeneratorType>::SampledSphericalField(
    ScalarFieldType field,
    SpherePointGeneratorType point_generator,
    size_t sample_count,
    size_t edge_samples
) :
    _field(std::move(field)),
    _weight_per_sample(sample_count > 0 ? UNIT_SPHERE_AREA / static_cast<double>(sample_count) : 0.0),
    _total_mass(0.0),
    _edge_samples(edge_samples) {
    build_samples(point_generator, sample_count);
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType>
void SampledSphericalField<ScalarFieldType, SpherePointGeneratorType>::build_samples(
    SpherePointGeneratorType& point_generator,
    size_t sample_count
) {
    SphericalBoundingBox full_sphere = SphericalBoundingBox::full_sphere();
    auto points = point_generator.generate(sample_count, full_sphere);

    _samples.reserve(points.size());
    double value_sum = 0.0;

    for (const auto& vector_point : points) {
        double sample_value = _field.value(vector_point);
        _samples.push_back({to_cgal_point(vector_point), sample_value});
        value_sum += sample_value;
    }

    _total_mass = value_sum * _weight_per_sample;
    build_kdtree();
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType>
void SampledSphericalField<ScalarFieldType, SpherePointGeneratorType>::build_kdtree() {
    if (_samples.empty()) return;

    _kdtree = std::make_unique<KDTree>();

    std::vector<Point3> points;
    points.reserve(_samples.size());
    for (const auto& sample : _samples) {
        points.push_back(sample.point);
    }
    _kdtree->insert(points.begin(), points.end());
    _kdtree->build();
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType>
double SampledSphericalField<ScalarFieldType, SpherePointGeneratorType>::value(
    const VectorS2& point
) const {
    return _field.value(point);
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType>
double SampledSphericalField<ScalarFieldType, SpherePointGeneratorType>::mass(
    const SphericalPolygon& polygon
) const {
    auto indices = find_samples_in_polygon(polygon);
    double value_sum = 0.0;
    for (size_t index : indices) {
        value_sum += _samples[index].value;
    }
    return value_sum * _weight_per_sample;
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType>
double SampledSphericalField<ScalarFieldType, SpherePointGeneratorType>::total_mass() const {
    return _total_mass;
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType>
double SampledSphericalField<ScalarFieldType, SpherePointGeneratorType>::edge_integral(
    const SphericalArc& arc
) const {
    double arc_length = arc.length();
    if (arc_length < 1e-10) return 0.0;

    double sum = 0.0;
    for (size_t i = 0; i < _edge_samples; ++i) {
        double t = (i + 0.5) / _edge_samples;
        VectorS2 point = interpolate(arc.source(), arc.target(), t);
        sum += _field.value(point);
    }

    return sum * arc_length / _edge_samples;
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType>
Eigen::Vector3d SampledSphericalField<ScalarFieldType, SpherePointGeneratorType>::edge_gradient_integral(
    const SphericalArc& arc
) const {
    double arc_length = arc.length();
    if (arc_length < 1e-10) return Eigen::Vector3d::Zero();

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < _edge_samples; ++i) {
        double t = (i + 0.5) / _edge_samples;
        VectorS2 interpolated = interpolate(arc.source(), arc.target(), t);
        double density = _field.value(interpolated);
        sum += density * interpolated;
    }

    return sum * arc_length / _edge_samples;
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType>
std::vector<size_t> SampledSphericalField<ScalarFieldType, SpherePointGeneratorType>::find_samples_in_polygon(
    const SphericalPolygon& polygon
) const {
    if (!_kdtree) return {};

    Point3 center = to_cgal_point(polygon.centroid());
    double radius = polygon.bounding_sphere_radius();
    FuzzySphere query(center, radius, GEOMETRIC_EPSILON);

    std::vector<Point3> candidates;
    _kdtree->search(std::back_inserter(candidates), query);

    std::vector<size_t> result;
    for (const auto& candidate : candidates) {
        if (polygon.contains(to_vector_s2(candidate))) {
            for (size_t i = 0; i < _samples.size(); ++i) {
                if (CGAL::squared_distance(_samples[i].point, candidate) < 1e-20) {
                    result.push_back(i);
                    break;
                }
            }
        }
    }
    return result;
}

static_assert(SphericalField<SampledSphericalField<NoiseField>>);

}

#endif //GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SAMPLED_SPHERICAL_FIELD_HPP_
