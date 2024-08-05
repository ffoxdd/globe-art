#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_CENTROID_CALCULATOR_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_CENTROID_CALCULATOR_HPP_

#include "../types.hpp"
#include "spherical_polygon.hpp"
#include "spherical_bounding_box.hpp"
#include "spherical_bounding_box_sampler.hpp"
#include "../noise_generator/noise_generator.hpp"
#include "../noise_generator/anl_noise_generator.hpp"
#include "../noise_generator/interval.hpp"

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
    // TODO: Inject the random stuff to avoid constantly reinitializing.
    SphericalBoundingBoxSampler _spherical_bounding_box_sampler;
    double _error_threshold;
    int _consecutive_stable_iterations_threshold;

    Point3 polygon_sample_point();
    Point3 bounding_box_sample_point();
    double density_at(const Point3 &point);
};

template<NoiseGenerator NG>
struct CentroidCalculator<NG>::Config {
    const SphericalPolygon &spherical_polygon;
    NG &noise_generator;
    double error_threshold = 01e-6;
    int consecutive_stable_iterations_threshold = 10;
};

template<NoiseGenerator NG>
CentroidCalculator<NG>::CentroidCalculator(CentroidCalculator::Config &&config):
    _spherical_polygon(config.spherical_polygon),
    _noise_generator(config.noise_generator),
    _spherical_bounding_box_sampler(SphericalBoundingBoxSampler()),
    _bounding_box(_spherical_polygon.bounding_box()),
    _error_threshold(config.error_threshold),
    _consecutive_stable_iterations_threshold(config.consecutive_stable_iterations_threshold) {
}

template<NoiseGenerator NG>
CentroidCalculator<NG>::CentroidCalculator() :
    CentroidCalculator(Config()) {
}

template<NoiseGenerator NG>
Point3 CentroidCalculator<NG>::centroid() {
    double total_weight = 0;
    Vector3 total_position(0, 0, 0);
    Vector3 previous_centroid(0, 0, 0);
    int consecutive_stable_iterations = 0;

    while (true) {
        if (total_weight != 0) {
            previous_centroid = total_position / total_weight;
        }

        Point3 sample_point = polygon_sample_point();
        double density = density_at(sample_point);

        total_position += position_vector(sample_point) * density;
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
double CentroidCalculator<NG>::density_at(const Point3 &point) {
    return _noise_generator.value(point);
}

template<NoiseGenerator NG>
Point3 CentroidCalculator<NG>::polygon_sample_point() {
    while (true) {
        Point3 sample_point = bounding_box_sample_point();

        if (_spherical_polygon.contains(sample_point)) {
            return sample_point;
        }
    }
}

template<NoiseGenerator NG>
Point3 CentroidCalculator<NG>::bounding_box_sample_point() {
    return _spherical_bounding_box_sampler.sample(_bounding_box);
}

}

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_CENTROID_CALCULATOR_HPP_
