#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_AREA_CALCULATOR_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_AREA_CALCULATOR_HPP_

#include "../types.hpp"
#include "spherical_polygon.hpp"
#include "spherical_bounding_box.hpp"
#include "sample_point_generator/sample_point_generator.hpp"
#include "sample_point_generator/bounding_box_sample_point_generator.hpp"
#include "../noise_generator/scalar_field.hpp"
#include "../noise_generator/anl_scalar_field.hpp"
#include <cmath>
#include <optional>
#include <utility>

namespace globe {

template<ScalarField DF = AnlScalarField, SamplePointGenerator SPG = BoundingBoxSamplePointGenerator>
class AreaCalculator {
 public:
    AreaCalculator(
        const SphericalPolygon &spherical_polygon,
        DF &density_field,
        SPG sample_point_generator,
        double error_threshold = 1e-6,
        int consecutive_stable_iterations_threshold = 10,
        std::optional<SphericalBoundingBox> bounding_box_override = std::nullopt
    );

    [[nodiscard]] double area();

 private:
    const SphericalPolygon &_spherical_polygon;
    DF &_density_field;
    const SphericalBoundingBox _bounding_box;
    double _error_threshold;
    int _consecutive_stable_iterations_threshold;
    SPG _sample_point_generator;

    double density_at(const Point3 &point);
    Point3 sample_point();
};

template<ScalarField DF, SamplePointGenerator SPG>
inline AreaCalculator<DF, SPG>::AreaCalculator(
    const SphericalPolygon &spherical_polygon,
    DF &density_field,
    SPG sample_point_generator,
    double error_threshold,
    int consecutive_stable_iterations_threshold,
    std::optional<SphericalBoundingBox> bounding_box_override
):
    _spherical_polygon(spherical_polygon),
    _density_field(density_field),
    _bounding_box(bounding_box_override.has_value()
        ? *bounding_box_override
        : _spherical_polygon.bounding_box()),
    _error_threshold(error_threshold),
    _consecutive_stable_iterations_threshold(consecutive_stable_iterations_threshold),
    _sample_point_generator(std::move(sample_point_generator)) {
}

template<ScalarField DF, SamplePointGenerator SPG>
inline double AreaCalculator<DF, SPG>::area() {

    int points_inside_polygon = 0;
    int total_points_sampled = 0;
    double total_mass = 0;
    int consecutive_stable_iterations = 0;
    double previous_weighted_area = 0;

    while (true) {
        Point3 sampled_point = sample_point();
        total_points_sampled++;

        if (_spherical_polygon.contains(sampled_point)) {
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
}

template<ScalarField DF, SamplePointGenerator SPG>
inline double AreaCalculator<DF, SPG>::density_at(const Point3 &point) {
    return _density_field.value(point);
}

template<ScalarField DF, SamplePointGenerator SPG>
inline Point3 AreaCalculator<DF, SPG>::sample_point() {
    return _sample_point_generator.generate();
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_AREA_CALCULATOR_HPP_
