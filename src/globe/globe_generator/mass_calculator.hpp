#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_MASS_CALCULATOR_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_MASS_CALCULATOR_HPP_

#include "../types.hpp"
#include "spherical_polygon.hpp"
#include "spherical_bounding_box.hpp"
#include "sample_point_generator/sample_point_generator.hpp"
#include "sample_point_generator/bounding_box_sample_point_generator.hpp"
#include "../scalar_field/scalar_field.hpp"
#include "../scalar_field/noise_field.hpp"
#include "monte_carlo_result.hpp"
#include <cmath>
#include <optional>
#include <utility>

namespace globe {

template<ScalarField DF = NoiseField, SamplePointGenerator SPG = BoundingBoxSamplePointGenerator>
class MassCalculator {
 public:
    MassCalculator(
        std::optional<std::reference_wrapper<const SphericalPolygon>> spherical_polygon,
        DF &density_field,
        SPG sample_point_generator,
        double error_threshold = 1e-6,
        int consecutive_stable_iterations_threshold = 10,
        std::optional<SphericalBoundingBox> bounding_box = std::nullopt,
        size_t max_samples = 1000000
    );

    [[nodiscard]] double mass();
    [[nodiscard]] MonteCarloResult compute();

 private:
    std::optional<std::reference_wrapper<const SphericalPolygon>> _spherical_polygon;
    DF &_density_field;
    const SphericalBoundingBox _bounding_box;
    double _error_threshold;
    int _consecutive_stable_iterations_threshold;
    size_t _max_samples;
    SPG _sample_point_generator;

    bool contains(const Point3 &point);
    double density_at(const Point3 &point);
    Point3 sample_point();

    static SphericalBoundingBox unit_sphere_bounding_box() {
        return SphericalBoundingBox(
            Interval(0, 2 * M_PI), Interval(-1, 1)
        );
    }
};

template<ScalarField DF, SamplePointGenerator SPG>
inline MassCalculator<DF, SPG>::MassCalculator(
    std::optional<std::reference_wrapper<const SphericalPolygon>> spherical_polygon,
    DF &density_field,
    SPG sample_point_generator,
    double error_threshold,
    int consecutive_stable_iterations_threshold,
    std::optional<SphericalBoundingBox> bounding_box,
    size_t max_samples
):
    _spherical_polygon(spherical_polygon),
    _density_field(density_field),
    _bounding_box(
        bounding_box.has_value() ?
        *bounding_box :
        (
            spherical_polygon.has_value() ? spherical_polygon->get().bounding_box()
            : unit_sphere_bounding_box()
        )
    ),
    _error_threshold(error_threshold),
    _consecutive_stable_iterations_threshold(consecutive_stable_iterations_threshold),
    _max_samples(max_samples),
    _sample_point_generator(std::move(sample_point_generator)) {
}

template<ScalarField DF, SamplePointGenerator SPG>
inline double MassCalculator<DF, SPG>::mass() {
    int points_inside_polygon = 0;
    int total_points_sampled = 0;
    double total_mass = 0;
    int consecutive_stable_iterations = 0;
    double previous_weighted_area = 0;

    while (total_points_sampled < _max_samples) {
        Point3 sampled_point = sample_point();
        total_points_sampled++;

        if (contains(sampled_point)) {
            points_inside_polygon++;
            total_mass += density_at(sampled_point);
        }

        if (points_inside_polygon == 0) {
            continue;
        }

        double inside_fraction = static_cast<double>(points_inside_polygon) / total_points_sampled;
        double spherical_polygon_area_estimate = _bounding_box.area() * inside_fraction;
        double average_density = total_mass / static_cast<double>(points_inside_polygon);
        double weighted_area_estimate = spherical_polygon_area_estimate * average_density;

        double error = std::pow(weighted_area_estimate - previous_weighted_area, 2);

        if (error < _error_threshold) {
            consecutive_stable_iterations++;
        } else {
            consecutive_stable_iterations = 0;
        }

        previous_weighted_area = weighted_area_estimate;

        if (consecutive_stable_iterations >= _consecutive_stable_iterations_threshold) {
            return weighted_area_estimate;
        }
    }

    if (points_inside_polygon == 0) {
        return 0.0;
    }

    double inside_fraction = static_cast<double>(points_inside_polygon) / total_points_sampled;
    double spherical_polygon_area_estimate = _bounding_box.area() * inside_fraction;
    double average_density = total_mass / static_cast<double>(points_inside_polygon);
    return spherical_polygon_area_estimate * average_density;
}

template<ScalarField DF, SamplePointGenerator SPG>
inline MonteCarloResult MassCalculator<DF, SPG>::compute() {
    int points_inside_polygon = 0;
    int total_points_sampled = 0;
    double sum_density = 0;
    double sum_density_squared = 0;
    int consecutive_stable_iterations = 0;
    double previous_weighted_area = 0;

    while (total_points_sampled < _max_samples) {
        Point3 sampled_point = sample_point();
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

        double error = std::pow(weighted_area_estimate - previous_weighted_area, 2);

        if (error < _error_threshold) {
            consecutive_stable_iterations++;
        } else {
            consecutive_stable_iterations = 0;
        }

        previous_weighted_area = weighted_area_estimate;

        if (consecutive_stable_iterations >= _consecutive_stable_iterations_threshold) {
            double mean_density = sum_density / points_inside_polygon;
            double mean_density_squared = sum_density_squared / points_inside_polygon;
            double variance = mean_density_squared - (mean_density * mean_density);

            return MonteCarloResult{
                weighted_area_estimate,
                variance,
                static_cast<size_t>(points_inside_polygon)
            };
        }
    }

    if (points_inside_polygon == 0) {
        return MonteCarloResult{0.0, 0.0, 0};
    }

    double inside_fraction = static_cast<double>(points_inside_polygon) / total_points_sampled;
    double spherical_polygon_area_estimate = _bounding_box.area() * inside_fraction;
    double average_density = sum_density / static_cast<double>(points_inside_polygon);
    double weighted_area_estimate = spherical_polygon_area_estimate * average_density;

    double mean_density = sum_density / points_inside_polygon;
    double mean_density_squared = sum_density_squared / points_inside_polygon;
    double variance = mean_density_squared - (mean_density * mean_density);

    return MonteCarloResult{
        weighted_area_estimate,
        variance,
        static_cast<size_t>(points_inside_polygon)
    };
}

template<ScalarField DF, SamplePointGenerator SPG>
inline bool MassCalculator<DF, SPG>::contains(const Point3 &point) {
    if (!_spherical_polygon) {
        return true;
    }

    return _spherical_polygon->get().contains(point);
}

template<ScalarField DF, SamplePointGenerator SPG>
inline double MassCalculator<DF, SPG>::density_at(const Point3 &point) {
    return _density_field.value(point);
}

template<ScalarField DF, SamplePointGenerator SPG>
inline Point3 MassCalculator<DF, SPG>::sample_point() {
    return _sample_point_generator.generate();
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_MASS_CALCULATOR_HPP_
