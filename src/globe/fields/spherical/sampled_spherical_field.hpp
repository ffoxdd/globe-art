#ifndef GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SAMPLED_SPHERICAL_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SAMPLED_SPHERICAL_FIELD_HPP_

#include "spherical_field.hpp"
#include "../../types.hpp"
#include "../../geometry/spherical/spherical_arc.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../geometry/spherical/spherical_bounding_box.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "../../generators/sphere_point_generator/sphere_point_generator.hpp"
#include "../../generators/sphere_point_generator/random_sphere_point_generator.hpp"
#include "../scalar/scalar_field.hpp"
#include "../scalar/noise_field.hpp"
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

    [[nodiscard]] double value(const Point3& point) const;
    [[nodiscard]] double mass(const SphericalPolygon& polygon) const;
    [[nodiscard]] double total_mass() const;
    [[nodiscard]] double edge_integral(const SphericalArc& arc) const;
    [[nodiscard]] Eigen::Vector3d edge_gradient_integral(const SphericalArc& arc) const;

 private:
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

    [[nodiscard]] Point3 interpolate_on_arc(
        const SphericalArc& arc,
        double t
    ) const;

    static Eigen::Vector3d to_eigen(const Point3& p);
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

    for (const auto& point : points) {
        double sample_value = _field.value(point);
        _samples.push_back({point, sample_value});
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
    const Point3& point
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
        Point3 point = interpolate_on_arc(arc, t);
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
        Point3 point = interpolate_on_arc(arc, t);
        double density = _field.value(point);
        sum += density * to_eigen(point);
    }

    return sum * arc_length / _edge_samples;
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType>
std::vector<size_t> SampledSphericalField<ScalarFieldType, SpherePointGeneratorType>::find_samples_in_polygon(
    const SphericalPolygon& polygon
) const {
    if (!_kdtree) return {};

    Point3 center = polygon.centroid();
    double radius = polygon.bounding_sphere_radius();
    FuzzySphere query(center, radius, GEOMETRIC_EPSILON);

    std::vector<Point3> candidates;
    _kdtree->search(std::back_inserter(candidates), query);

    std::vector<size_t> result;
    for (const auto& candidate : candidates) {
        if (polygon.contains(candidate)) {
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

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType>
Point3 SampledSphericalField<ScalarFieldType, SpherePointGeneratorType>::interpolate_on_arc(
    const SphericalArc& arc,
    double t
) const {
    Eigen::Vector3d source = to_eigen(arc.source());
    Eigen::Vector3d target = to_eigen(arc.target());

    double theta = arc.angle();
    if (theta < 1e-10) {
        return arc.source();
    }

    double sin_theta = std::sin(theta);
    double a = std::sin((1.0 - t) * theta) / sin_theta;
    double b = std::sin(t * theta) / sin_theta;

    Eigen::Vector3d result = a * source + b * target;
    result.normalize();

    return Point3(result.x(), result.y(), result.z());
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType>
Eigen::Vector3d SampledSphericalField<ScalarFieldType, SpherePointGeneratorType>::to_eigen(
    const Point3& p
) {
    return Eigen::Vector3d(p.x(), p.y(), p.z());
}

static_assert(SphericalField<SampledSphericalField<NoiseField>>);

}

#endif //GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SAMPLED_SPHERICAL_FIELD_HPP_
