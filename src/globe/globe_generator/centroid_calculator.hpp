#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_CENTROID_CALCULATOR_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_CENTROID_CALCULATOR_HPP_

#include "../types.hpp"
#include "spherical_polygon.hpp"
#include "spherical_bounding_box.hpp"
#include "spherical_bounding_box_sampler.hpp"
#include "../noise_generator/noise_generator.hpp"
#include "../noise_generator/anl_noise_generator.hpp"
#include "../noise_generator/interval.hpp"
#include <functional>
#include <optional>
#include <utility>

namespace globe {

template<NoiseGenerator NG = AnlNoiseGenerator>
class CentroidCalculator {
 public:
    struct Config;
    CentroidCalculator();
    explicit CentroidCalculator(Config &&config);

    [[nodiscard]] Point3 centroid();

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
struct CentroidCalculator<NG>::Config {
    const SphericalPolygon &spherical_polygon;
    NG &noise_generator;
    double error_threshold = 1e-6;
    int consecutive_stable_iterations_threshold = 10;
    std::function<Point3()> sample_point_generator = {};
    std::optional<SphericalBoundingBox> bounding_box_override = std::nullopt;
};

template<NoiseGenerator NG>
inline CentroidCalculator<NG>::CentroidCalculator(CentroidCalculator::Config &&config):
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
inline CentroidCalculator<NG>::CentroidCalculator() :
    CentroidCalculator(Config()) {
}

template<NoiseGenerator NG>
inline Point3 CentroidCalculator<NG>::centroid() {
    double total_weight = 0;
    Vector3 total_position(0, 0, 0);
    Vector3 previous_centroid(0, 0, 0);
    int consecutive_stable_iterations = 0;

    while (true) {
        if (total_weight != 0) {
            previous_centroid = total_position / total_weight;
        }

        Point3 sampled_point;
        do {
            sampled_point = sample_point();
        } while (!_spherical_polygon.contains(sampled_point));

        double density = density_at(sampled_point);

        total_position += position_vector(sampled_point) * density;
        total_weight += density;

        Vector3 current_centroid = total_position / total_weight;
        double error = (current_centroid - previous_centroid).squared_length();

        if (error < _error_threshold) {
            consecutive_stable_iterations++;
        } else {
            consecutive_stable_iterations = 0;
        }

        if (consecutive_stable_iterations >= _consecutive_stable_iterations_threshold) {
            Point3 point = to_point(current_centroid);
            return project_to_sphere(point);
        }
    }
}

template<NoiseGenerator NG>
inline double CentroidCalculator<NG>::density_at(const Point3 &point) {
    return _noise_generator.value(point);
}

template<NoiseGenerator NG>
inline Point3 CentroidCalculator<NG>::sample_point() {
    return _sample_point_generator();
}

}

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_CENTROID_CALCULATOR_HPP_
