#ifndef GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SAMPLED_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SAMPLED_FIELD_HPP_

#include "field.hpp"
#include "../../cgal_types.hpp"
#include "../../geometry/spherical/arc.hpp"
#include "../../geometry/spherical/polygon/polygon.hpp"
#include "../../geometry/spherical/bounding_box.hpp"
#include "../../generators/spherical/point_generator.hpp"
#include "../../generators/spherical/random_point_generator.hpp"
#include "../scalar/field.hpp"
#include "../scalar/noise_field.hpp"
#include <Eigen/Core>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

namespace globe::fields::spherical {

using geometry::spherical::UNIT_SPHERE_AREA;

template<
    scalar::Field ScalarFieldType,
    generators::spherical::PointGenerator SpherePointGeneratorType = generators::spherical::RandomPointGenerator<>
>
class SampledField {
 public:
    SampledField(
        ScalarFieldType field,
        SpherePointGeneratorType point_generator = SpherePointGeneratorType(),
        size_t sample_count = 200'000,
        size_t edge_samples = 50
    );

    SampledField(const SampledField&) = delete;
    SampledField& operator=(const SampledField&) = delete;
    SampledField(SampledField&&) = default;
    SampledField& operator=(SampledField&&) = default;

    [[nodiscard]] double value(const VectorS2& point) const;
    [[nodiscard]] double mass(const Polygon& polygon) const;
    [[nodiscard]] double total_mass() const;
    [[nodiscard]] double edge_integral(const Arc& arc) const;
    [[nodiscard]] Eigen::Vector3d edge_gradient_integral(const Arc& arc) const;

 private:
    struct Sample {
        cgal::Point3 point;
        double value;
    };

    ScalarFieldType _field;
    std::vector<Sample> _samples;
    std::unique_ptr<cgal::KDTree> _kdtree;
    double _weight_per_sample;
    double _total_mass;
    size_t _edge_samples;

    void build_samples(
        SpherePointGeneratorType& point_generator,
        size_t sample_count
    );
    void build_kdtree();

    [[nodiscard]] std::vector<size_t> find_samples_in_polygon(
        const Polygon& polygon
    ) const;
};

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType>
SampledField<ScalarFieldType, SpherePointGeneratorType>::SampledField(
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

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType>
void SampledField<ScalarFieldType, SpherePointGeneratorType>::build_samples(
    SpherePointGeneratorType& point_generator,
    size_t sample_count
) {
    SphericalBoundingBox full_sphere = SphericalBoundingBox::full_sphere();
    auto points = point_generator.generate(sample_count, full_sphere);

    _samples.reserve(points.size());
    double value_sum = 0.0;

    for (const auto& vector_point : points) {
        double sample_value = _field.value(vector_point);
        _samples.push_back({cgal::to_point(vector_point), sample_value});
        value_sum += sample_value;
    }

    _total_mass = value_sum * _weight_per_sample;
    build_kdtree();
}

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType>
void SampledField<ScalarFieldType, SpherePointGeneratorType>::build_kdtree() {
    if (_samples.empty()) return;

    _kdtree = std::make_unique<cgal::KDTree>();

    std::vector<cgal::Point3> points;
    points.reserve(_samples.size());
    for (const auto& sample : _samples) {
        points.push_back(sample.point);
    }
    _kdtree->insert(points.begin(), points.end());
    _kdtree->build();
}

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType>
double SampledField<ScalarFieldType, SpherePointGeneratorType>::value(
    const VectorS2& point
) const {
    return _field.value(point);
}

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType>
double SampledField<ScalarFieldType, SpherePointGeneratorType>::mass(
    const Polygon& polygon
) const {
    auto indices = find_samples_in_polygon(polygon);
    double value_sum = 0.0;
    for (size_t index : indices) {
        value_sum += _samples[index].value;
    }
    return value_sum * _weight_per_sample;
}

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType>
double SampledField<ScalarFieldType, SpherePointGeneratorType>::total_mass() const {
    return _total_mass;
}

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType>
double SampledField<ScalarFieldType, SpherePointGeneratorType>::edge_integral(
    const Arc& arc
) const {
    double arc_length = arc.length();
    if (arc_length < 1e-10) return 0.0;

    double sum = 0.0;
    for (size_t i = 0; i < _edge_samples; ++i) {
        double t = (i + 0.5) / _edge_samples;
        VectorS2 point = arc.interpolate(t);
        sum += _field.value(point);
    }

    return sum * arc_length / _edge_samples;
}

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType>
Eigen::Vector3d SampledField<ScalarFieldType, SpherePointGeneratorType>::edge_gradient_integral(
    const Arc& arc
) const {
    double arc_length = arc.length();
    if (arc_length < 1e-10) return Eigen::Vector3d::Zero();

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < _edge_samples; ++i) {
        double t = (i + 0.5) / _edge_samples;
        VectorS2 point = arc.interpolate(t);
        double density = _field.value(point);
        sum += density * point;
    }

    return sum * arc_length / _edge_samples;
}

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType>
std::vector<size_t> SampledField<ScalarFieldType, SpherePointGeneratorType>::find_samples_in_polygon(
    const Polygon& polygon
) const {
    if (!_kdtree) return {};

    cgal::Point3 center = cgal::to_point(polygon.centroid());
    double radius = polygon.bounding_sphere_radius();
    cgal::FuzzySphere query(center, radius, GEOMETRIC_EPSILON);

    std::vector<cgal::Point3> candidates;
    _kdtree->search(std::back_inserter(candidates), query);

    std::vector<size_t> result;
    for (const auto& candidate : candidates) {
        if (polygon.contains(to_vector_s2(candidate))) {
            for (size_t i = 0; i < _samples.size(); ++i) {
                if (::CGAL::squared_distance(_samples[i].point, candidate) < 1e-20) {
                    result.push_back(i);
                    break;
                }
            }
        }
    }
    return result;
}

static_assert(Field<SampledField<scalar::NoiseField>>);

} // namespace globe::fields::spherical

#endif //GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_SAMPLED_FIELD_HPP_
