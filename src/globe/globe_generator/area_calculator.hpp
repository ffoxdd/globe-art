#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_AREA_CALCULATOR_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_AREA_CALCULATOR_HPP_

#include "../types.hpp"
#include "spherical_polygon.hpp"
#include "spherical_bounding_box.hpp"
#include "spherical_bounding_box_sampler.hpp"
#include "../noise_generator/noise_generator.hpp"
#include "../noise_generator/anl_noise_generator.hpp"
#include "../noise_generator/interval.hpp"
#include <cmath>
#include <functional>
#include <optional>
#include <utility>

namespace globe {

template<NoiseGenerator NG = AnlNoiseGenerator>
class AreaCalculator {
 public:
    struct Config;
    AreaCalculator();
    explicit AreaCalculator(Config &&config);

    [[nodiscard]] double area();

 private:
    const SphericalPolygon &_spherical_polygon;
    NG &_noise_generator;
    const SphericalBoundingBox _bounding_box;
    SphericalBoundingBoxSampler _spherical_bounding_box_sampler;
    double _error_threshold;
    int _consecutive_stable_iterations_threshold;
    std::function<Point3()> _sample_point_generator;

    double density_at(const Point3 &point);
    Point3 sample_point();
};

template<NoiseGenerator NG>
struct AreaCalculator<NG>::Config {
    const SphericalPolygon &spherical_polygon;
    NG &noise_generator;
    double error_threshold = 1e-6;
    int consecutive_stable_iterations_threshold = 10;
    std::function<Point3()> sample_point_generator = {};
    std::optional<SphericalBoundingBox> bounding_box_override = std::nullopt;
};

template<NoiseGenerator NG>
inline AreaCalculator<NG>::AreaCalculator(AreaCalculator::Config &&config):
    _spherical_polygon(config.spherical_polygon),
    _noise_generator(config.noise_generator),
    _bounding_box(config.bounding_box_override.has_value()
        ? *config.bounding_box_override
        : _spherical_polygon.bounding_box()),
    _spherical_bounding_box_sampler(SphericalBoundingBoxSampler()),
    _error_threshold(config.error_threshold),
    _consecutive_stable_iterations_threshold(config.consecutive_stable_iterations_threshold) {

    if (config.sample_point_generator) {
        _sample_point_generator = std::move(config.sample_point_generator);
    } else {
        _sample_point_generator = [this]() {
            return _spherical_bounding_box_sampler.sample(_bounding_box);
        };
    }
}

template<NoiseGenerator NG>
inline AreaCalculator<NG>::AreaCalculator() :
    AreaCalculator(Config()) {
}

template<NoiseGenerator NG>
inline double AreaCalculator<NG>::area() {

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

template<NoiseGenerator NG>
inline double AreaCalculator<NG>::density_at(const Point3 &point) {
    return _noise_generator.value(point);
}

template<NoiseGenerator NG>
inline Point3 AreaCalculator<NG>::sample_point() {
    return _sample_point_generator();
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_AREA_CALCULATOR_HPP_
