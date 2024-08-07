#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_AREA_CALCULATOR_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_AREA_CALCULATOR_HPP_

#include "../types.hpp"
#include "spherical_polygon.hpp"
#include "spherical_bounding_box.hpp"
#include "spherical_bounding_box_sampler.hpp"
#include "../noise_generator/noise_generator.hpp"
#include "../noise_generator/anl_noise_generator.hpp"
#include "../noise_generator/interval.hpp"

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

    Point3 bounding_box_sample_point();
    double density_at(const Point3 &point);
};

template<NoiseGenerator NG>
struct AreaCalculator<NG>::Config {
    const SphericalPolygon &spherical_polygon;
    NG &noise_generator;
    double error_threshold = 01e-6;
    int consecutive_stable_iterations_threshold = 10;
};

template<NoiseGenerator NG>
AreaCalculator<NG>::AreaCalculator(AreaCalculator::Config &&config):
    _spherical_polygon(config.spherical_polygon),
    _noise_generator(config.noise_generator),
    _spherical_bounding_box_sampler(SphericalBoundingBoxSampler()),
    _bounding_box(_spherical_polygon.bounding_box()),
    _error_threshold(config.error_threshold),
    _consecutive_stable_iterations_threshold(config.consecutive_stable_iterations_threshold) {
}

template<NoiseGenerator NG>
AreaCalculator<NG>::AreaCalculator() :
    AreaCalculator(Config()) {
}

template<NoiseGenerator NG>
double AreaCalculator<NG>::area() {
    double total_weighted_area = 0;
    double total_density_sum = 0;
    int points_inside_polygon = 0;
    int total_points_sampled = 0;
    int consecutive_stable_iterations = 0;
    double previous_weighted_area = 0;

    while (true) {
        Point3 sample_point = bounding_box_sample_point();
        total_points_sampled++;

        if (_spherical_polygon.contains(sample_point)) {
            points_inside_polygon++;
            double density = density_at(sample_point);
            total_weighted_area += density;
            total_density_sum += 1;
        }

        double spherical_polygon_area_estimate =
            _bounding_box.area() * (static_cast<double>(points_inside_polygon) / total_points_sampled);

        double weighted_area_estimate =
            spherical_polygon_area_estimate * (total_weighted_area / total_density_sum);

        double error = std::abs(weighted_area_estimate - previous_weighted_area);

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
double AreaCalculator<NG>::density_at(const Point3 &point) {
    return _noise_generator.value(point);
}

template<NoiseGenerator NG>
Point3 AreaCalculator<NG>::bounding_box_sample_point() {
    return _spherical_bounding_box_sampler.sample(_bounding_box);
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_AREA_CALCULATOR_HPP_
