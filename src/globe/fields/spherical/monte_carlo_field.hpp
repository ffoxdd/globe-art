#ifndef GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_MONTE_CARLO_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_MONTE_CARLO_FIELD_HPP_

#include "field.hpp"
#include "../../cgal_types.hpp"
#include "../../geometry/spherical/arc.hpp"
#include "../../geometry/spherical/polygon/polygon.hpp"
#include "../../geometry/spherical/bounding_box.hpp"
#include "../../generators/spherical/point_generator.hpp"
#include "../../generators/spherical/random_point_generator.hpp"
#include "../../math/interval_sampler/interval_sampler.hpp"
#include "../scalar/field.hpp"
#include "../scalar/noise_field.hpp"
#include <Eigen/Core>
#include <cmath>

namespace globe::fields::spherical {

using geometry::spherical::UNIT_SPHERE_AREA;

template<
    scalar::Field ScalarFieldType,
    generators::spherical::PointGenerator SpherePointGeneratorType = generators::spherical::RandomPointGenerator<>,
    IntervalSampler IntervalSamplerType = UniformIntervalSampler
>
class MonteCarloField {
 public:
    MonteCarloField(
        ScalarFieldType field,
        SpherePointGeneratorType point_generator = SpherePointGeneratorType(),
        IntervalSamplerType interval_sampler = IntervalSamplerType()
    );

    [[nodiscard]] double value(const VectorS2& point) const;
    [[nodiscard]] double mass(const Polygon& polygon) const;
    [[nodiscard]] double total_mass() const;
    [[nodiscard]] double edge_integral(const Arc& arc) const;
    [[nodiscard]] Eigen::Vector3d edge_gradient_integral(const Arc& arc) const;

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
};

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType, IntervalSampler IntervalSamplerType>
MonteCarloField<ScalarFieldType, SpherePointGeneratorType, IntervalSamplerType>::MonteCarloField(
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

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType, IntervalSampler IntervalSamplerType>
double MonteCarloField<ScalarFieldType, SpherePointGeneratorType, IntervalSamplerType>::value(
    const VectorS2& point
) const {
    return _field.value(point);
}

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType, IntervalSampler IntervalSamplerType>
double MonteCarloField<ScalarFieldType, SpherePointGeneratorType, IntervalSamplerType>::mass(
    const Polygon& polygon
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

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType, IntervalSampler IntervalSamplerType>
double MonteCarloField<ScalarFieldType, SpherePointGeneratorType, IntervalSamplerType>::total_mass() const {
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

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType, IntervalSampler IntervalSamplerType>
double MonteCarloField<ScalarFieldType, SpherePointGeneratorType, IntervalSamplerType>::edge_integral(
    const Arc& arc
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
            VectorS2 point = arc.interpolate(t);
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

template<scalar::Field ScalarFieldType, generators::spherical::PointGenerator SpherePointGeneratorType, IntervalSampler IntervalSamplerType>
Eigen::Vector3d MonteCarloField<ScalarFieldType, SpherePointGeneratorType, IntervalSamplerType>::edge_gradient_integral(
    const Arc& arc
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
            VectorS2 point = arc.interpolate(t);
            double density = _field.value(point);
            weighted_sum += density * point;
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

static_assert(Field<MonteCarloField<scalar::NoiseField>>);

} // namespace globe::fields::spherical

#endif //GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_MONTE_CARLO_FIELD_HPP_
