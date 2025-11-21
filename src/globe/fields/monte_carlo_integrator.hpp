#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_MONTE_CARLO_INTEGRATOR_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_MONTE_CARLO_INTEGRATOR_HPP_

#include "../types.hpp"
#include "../spherical/spherical_polygon.hpp"
#include "../spherical/spherical_bounding_box.hpp"
#include "../sampling/sample_point_generator/sample_point_generator.hpp"
#include "../sampling/sample_point_generator/bounding_box_sample_point_generator.hpp"
#include "scalar_field.hpp"
#include "noise_field.hpp"
#include <cmath>
#include <optional>
#include <cstddef>
#include <utility>

namespace globe {

template<ScalarField DF = NoiseField, SamplePointGenerator SPG = BoundingBoxSamplePointGenerator>
class MonteCarloIntegrator {
 public:
    MonteCarloIntegrator(
        std::optional<std::reference_wrapper<const SphericalPolygon>> spherical_polygon,
        DF &density_field,
        SPG sample_point_generator = SPG(),
        std::optional<SphericalBoundingBox> bounding_box = std::nullopt
    );

    [[nodiscard]] double integrate();

 private:
    std::optional<std::reference_wrapper<const SphericalPolygon>> _spherical_polygon;
    DF &_density_field;
    SPG _sample_point_generator;
    SphericalBoundingBox _bounding_box;

    static constexpr double RELATIVE_CHANGE_THRESHOLD = 0.001;
    static constexpr size_t MAX_SAMPLES_GUARD = 3'000'000;
    static constexpr int CONSECUTIVE_STABLE_ITERATIONS = 15;
    static constexpr size_t MIN_HITS = 2000;

    static SphericalBoundingBox unit_sphere_bounding_box() {
        return SphericalBoundingBox(
            Interval(0, 2 * M_PI), Interval(-1, 1)
        );
    }
};

template<ScalarField DF, SamplePointGenerator SPG>
inline MonteCarloIntegrator<DF, SPG>::MonteCarloIntegrator(
    std::optional<std::reference_wrapper<const SphericalPolygon>> spherical_polygon,
    DF &density_field,
    SPG sample_point_generator,
    std::optional<SphericalBoundingBox> bounding_box
):
    _spherical_polygon(spherical_polygon),
    _density_field(density_field),
    _sample_point_generator(std::move(sample_point_generator)),
    _bounding_box(
        bounding_box.has_value() ?
        *bounding_box :
        (
            spherical_polygon.has_value() ? spherical_polygon->get().bounding_box()
            : unit_sphere_bounding_box()
        )
    ) {
}

template<ScalarField DF, SamplePointGenerator SPG>
inline double MonteCarloIntegrator<DF, SPG>::integrate() {
    auto contains = [&](const Point3 &point) {
        if (!_spherical_polygon) {
            return true;
        }

        return _spherical_polygon->get().contains(point);
    };

    auto density_at = [&](const Point3 &point) {
        return _density_field.value(point);
    };

    int points_inside_polygon = 0;
    int total_points_sampled = 0;
    double sum_density = 0;
    double sum_density_squared = 0;
    int consecutive_stable_iterations = 0;
    double previous_weighted_area = 0;

    while (static_cast<size_t>(total_points_sampled) < MAX_SAMPLES_GUARD) {
        Point3 sampled_point = _sample_point_generator.generate(_bounding_box);
        total_points_sampled++;

        if (contains(sampled_point)) {
            points_inside_polygon++;
            double density = density_at(sampled_point);
            sum_density += density;
            sum_density_squared += density * density;
        }

        if (points_inside_polygon == 0) {
            continue;
        }

        double inside_fraction = static_cast<double>(points_inside_polygon) / total_points_sampled;
        double spherical_polygon_area_estimate = _bounding_box.area() * inside_fraction;
        double average_density = sum_density / static_cast<double>(points_inside_polygon);
        double weighted_area_estimate = spherical_polygon_area_estimate * average_density;

        double relative_change = (previous_weighted_area > 0) ?
            std::abs(weighted_area_estimate - previous_weighted_area) / previous_weighted_area :
            1.0;

        if (relative_change < RELATIVE_CHANGE_THRESHOLD) {
            consecutive_stable_iterations++;
        } else {
            consecutive_stable_iterations = 0;
        }

        previous_weighted_area = weighted_area_estimate;

        if (
            points_inside_polygon >= static_cast<int>(MIN_HITS) &&
            consecutive_stable_iterations >= CONSECUTIVE_STABLE_ITERATIONS
        ) {
            double mean_density = sum_density / points_inside_polygon;
            double mean_density_squared = sum_density_squared / points_inside_polygon;
            double variance = mean_density_squared - (mean_density * mean_density);

            return weighted_area_estimate;
        }
    }

    if (points_inside_polygon == 0) {
        return 0.0;
    }

    double inside_fraction = static_cast<double>(points_inside_polygon) / total_points_sampled;
    double spherical_polygon_area_estimate = _bounding_box.area() * inside_fraction;
    double average_density = sum_density / static_cast<double>(points_inside_polygon);
    double weighted_area_estimate = spherical_polygon_area_estimate * average_density;

    return weighted_area_estimate;
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_MONTE_CARLO_INTEGRATOR_HPP_
