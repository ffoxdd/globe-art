#ifndef GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_MONTE_CARLO_SPHERICAL_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_MONTE_CARLO_SPHERICAL_FIELD_HPP_

#include "spherical_field.hpp"
#include "../../types.hpp"
#include "../../geometry/spherical/spherical_arc.hpp"
#include "../../geometry/spherical/spherical_polygon/spherical_polygon.hpp"
#include "../../geometry/spherical/spherical_bounding_box.hpp"
#include "../../generators/sphere_point_generator/sphere_point_generator.hpp"
#include "../../generators/sphere_point_generator/random_sphere_point_generator.hpp"
#include "../../math/interval_sampler/interval_sampler.hpp"
#include "../scalar/scalar_field.hpp"
#include "../scalar/noise_field.hpp"
#include <Eigen/Core>
#include <cmath>

namespace globe {

template<
    ScalarField ScalarFieldType,
    SpherePointGenerator SpherePointGeneratorType = RandomSpherePointGenerator<>,
    IntervalSampler IntervalSamplerType = UniformIntervalSampler
>
class MonteCarloSphericalField {
 public:
    MonteCarloSphericalField(
        ScalarFieldType field,
        SpherePointGeneratorType point_generator = SpherePointGeneratorType(),
        IntervalSamplerType interval_sampler = IntervalSamplerType()
    );

    [[nodiscard]] double value(const Point3& point) const;
    [[nodiscard]] double mass(const SphericalPolygon& polygon) const;
    [[nodiscard]] double total_mass() const;
    [[nodiscard]] double edge_integral(const SphericalArc& arc) const;
    [[nodiscard]] Eigen::Vector3d edge_gradient_integral(const SphericalArc& arc) const;

 private:
    ScalarFieldType _field;
    mutable SpherePointGeneratorType _point_generator;
    mutable IntervalSamplerType _interval_sampler;
    mutable double _cached_total_mass;
    mutable bool _total_mass_computed;

    static constexpr size_t MASS_SAMPLE_BATCH = 1000;
    static constexpr size_t MAX_MASS_SAMPLES = 1'000'000;
    static constexpr double RELATIVE_TOLERANCE = 0.005;
    static constexpr int STABLE_ITERATIONS_REQUIRED = 10;

    [[nodiscard]] Point3 interpolate_on_arc(
        const SphericalArc& arc,
        double t
    ) const;
};

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType, IntervalSampler IntervalSamplerType>
MonteCarloSphericalField<ScalarFieldType, SpherePointGeneratorType, IntervalSamplerType>::MonteCarloSphericalField(
    ScalarFieldType field,
    SpherePointGeneratorType point_generator,
    IntervalSamplerType interval_sampler
) :
    _field(std::move(field)),
    _point_generator(std::move(point_generator)),
    _interval_sampler(std::move(interval_sampler)),
    _cached_total_mass(0.0),
    _total_mass_computed(false) {
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType, IntervalSampler IntervalSamplerType>
double MonteCarloSphericalField<ScalarFieldType, SpherePointGeneratorType, IntervalSamplerType>::value(
    const Point3& point
) const {
    return _field.value(point);
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType, IntervalSampler IntervalSamplerType>
double MonteCarloSphericalField<ScalarFieldType, SpherePointGeneratorType, IntervalSamplerType>::mass(
    const SphericalPolygon& polygon
) const {
    SphericalBoundingBox bounding_box = polygon.bounding_box();

    size_t total_samples = 0;
    size_t hits = 0;
    double density_sum = 0.0;
    double previous_estimate = 0.0;
    int stable_iterations = 0;

    while (total_samples < MAX_MASS_SAMPLES) {
        auto batch = _point_generator.generate(MASS_SAMPLE_BATCH, bounding_box);

        for (const auto& point : batch) {
            total_samples++;
            if (polygon.contains(point)) {
                hits++;
                density_sum += _field.value(point);
            }
        }

        if (hits == 0) continue;

        double hit_fraction = static_cast<double>(hits) / total_samples;
        double polygon_area_estimate = bounding_box.area() * hit_fraction;
        double average_density = density_sum / hits;
        double mass_estimate = polygon_area_estimate * average_density;

        if (previous_estimate > 0) {
            double relative_change = std::abs(mass_estimate - previous_estimate) / previous_estimate;
            if (relative_change < RELATIVE_TOLERANCE) {
                stable_iterations++;
                if (stable_iterations >= STABLE_ITERATIONS_REQUIRED && hits >= 100) {
                    return mass_estimate;
                }
            } else {
                stable_iterations = 0;
            }
        }

        previous_estimate = mass_estimate;
    }

    if (hits == 0) return 0.0;

    double hit_fraction = static_cast<double>(hits) / total_samples;
    double polygon_area_estimate = bounding_box.area() * hit_fraction;
    double average_density = density_sum / hits;
    return polygon_area_estimate * average_density;
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType, IntervalSampler IntervalSamplerType>
double MonteCarloSphericalField<ScalarFieldType, SpherePointGeneratorType, IntervalSamplerType>::total_mass() const {
    if (_total_mass_computed) {
        return _cached_total_mass;
    }

    SphericalBoundingBox full_sphere = SphericalBoundingBox::full_sphere();

    size_t total_samples = 0;
    double density_sum = 0.0;
    double previous_estimate = 0.0;
    int stable_iterations = 0;

    while (total_samples < MAX_MASS_SAMPLES) {
        auto batch = _point_generator.generate(MASS_SAMPLE_BATCH, full_sphere);

        for (const auto& point : batch) {
            total_samples++;
            density_sum += _field.value(point);
        }

        double average_density = density_sum / total_samples;
        double mass_estimate = UNIT_SPHERE_AREA * average_density;

        if (previous_estimate > 0) {
            double relative_change = std::abs(mass_estimate - previous_estimate) / previous_estimate;
            if (relative_change < RELATIVE_TOLERANCE) {
                stable_iterations++;
                if (stable_iterations >= STABLE_ITERATIONS_REQUIRED) {
                    _cached_total_mass = mass_estimate;
                    _total_mass_computed = true;
                    return _cached_total_mass;
                }
            } else {
                stable_iterations = 0;
            }
        }

        previous_estimate = mass_estimate;
    }

    _cached_total_mass = UNIT_SPHERE_AREA * (density_sum / total_samples);
    _total_mass_computed = true;
    return _cached_total_mass;
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType, IntervalSampler IntervalSamplerType>
double MonteCarloSphericalField<ScalarFieldType, SpherePointGeneratorType, IntervalSamplerType>::edge_integral(
    const SphericalArc& arc
) const {
    double arc_length = arc.length();
    if (arc_length < 1e-10) return 0.0;

    static constexpr size_t EDGE_BATCH_SIZE = 100;
    static constexpr size_t MAX_EDGE_SAMPLES = 10'000;

    size_t total_samples = 0;
    double value_sum = 0.0;
    double previous_estimate = 0.0;
    int stable_iterations = 0;

    while (total_samples < MAX_EDGE_SAMPLES) {
        for (size_t i = 0; i < EDGE_BATCH_SIZE; ++i) {
            double t = _interval_sampler.sample(UNIT_INTERVAL);
            Point3 point = interpolate_on_arc(arc, t);
            value_sum += _field.value(point);
            total_samples++;
        }

        double average_value = value_sum / total_samples;
        double estimate = average_value * arc_length;

        if (previous_estimate > 0) {
            double relative_change = std::abs(estimate - previous_estimate) / previous_estimate;
            if (relative_change < RELATIVE_TOLERANCE) {
                stable_iterations++;
                if (stable_iterations >= STABLE_ITERATIONS_REQUIRED) {
                    return estimate;
                }
            } else {
                stable_iterations = 0;
            }
        }

        previous_estimate = estimate;
    }

    return (value_sum / total_samples) * arc_length;
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType, IntervalSampler IntervalSamplerType>
Eigen::Vector3d MonteCarloSphericalField<ScalarFieldType, SpherePointGeneratorType, IntervalSamplerType>::edge_gradient_integral(
    const SphericalArc& arc
) const {
    double arc_length = arc.length();
    if (arc_length < 1e-10) return Eigen::Vector3d::Zero();

    static constexpr size_t EDGE_BATCH_SIZE = 100;
    static constexpr size_t MAX_EDGE_SAMPLES = 10'000;

    size_t total_samples = 0;
    Eigen::Vector3d weighted_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_estimate = Eigen::Vector3d::Zero();
    int stable_iterations = 0;

    while (total_samples < MAX_EDGE_SAMPLES) {
        for (size_t i = 0; i < EDGE_BATCH_SIZE; ++i) {
            double t = _interval_sampler.sample(UNIT_INTERVAL);
            Point3 point = interpolate_on_arc(arc, t);
            double density = _field.value(point);
            weighted_sum += density * Eigen::Vector3d(point.x(), point.y(), point.z());
            total_samples++;
        }

        Eigen::Vector3d average = weighted_sum / total_samples;
        Eigen::Vector3d estimate = average * arc_length;

        if (previous_estimate.norm() > 0) {
            double relative_change = (estimate - previous_estimate).norm() / previous_estimate.norm();
            if (relative_change < RELATIVE_TOLERANCE) {
                stable_iterations++;
                if (stable_iterations >= STABLE_ITERATIONS_REQUIRED) {
                    return estimate;
                }
            } else {
                stable_iterations = 0;
            }
        }

        previous_estimate = estimate;
    }

    return (weighted_sum / total_samples) * arc_length;
}

template<ScalarField ScalarFieldType, SpherePointGenerator SpherePointGeneratorType, IntervalSampler IntervalSamplerType>
Point3 MonteCarloSphericalField<ScalarFieldType, SpherePointGeneratorType, IntervalSamplerType>::interpolate_on_arc(
    const SphericalArc& arc,
    double t
) const {
    Eigen::Vector3d source(arc.source().x(), arc.source().y(), arc.source().z());
    Eigen::Vector3d target(arc.target().x(), arc.target().y(), arc.target().z());

    double theta = arc.length();
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

static_assert(SphericalField<MonteCarloSphericalField<NoiseField>>);

}

#endif //GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_MONTE_CARLO_SPHERICAL_FIELD_HPP_
