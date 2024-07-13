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

    Point3 sample_polygon();
    Point3 sample_bounding_box();
    double density_at(const Point3 &point);
};

template<NoiseGenerator NG>
struct CentroidCalculator<NG>::Config {
    const SphericalPolygon &spherical_polygon;
    NG &noise_generator;
};

template<NoiseGenerator NG>
CentroidCalculator<NG>::CentroidCalculator(CentroidCalculator::Config &&config):
    _spherical_polygon(config.spherical_polygon),
    _noise_generator(config.noise_generator),
    _spherical_bounding_box_sampler(SphericalBoundingBoxSampler()),
    _bounding_box(_spherical_polygon.bounding_box()) {
}

template<NoiseGenerator NG>
CentroidCalculator<NG>::CentroidCalculator() :
    CentroidCalculator(Config()) {
}

template<NoiseGenerator NG>
Point3 CentroidCalculator<NG>::centroid() {
    int iterations = 3;
    double total_weight = 0;
    Vector3 total_position(0, 0, 0);

    for (int n = 0; n < iterations; n++) {
        Point3 sample_point = sample_polygon();
        double density = density_at(sample_point);

        total_position += position_vector(sample_point) * density;
        total_weight += density;
    }

    auto centroid = to_point(total_position / total_weight);
    return project_to_sphere(centroid);
}

template<NoiseGenerator NG>
double CentroidCalculator<NG>::density_at(const Point3 &point) {
    return _noise_generator.value(point);
}

template<NoiseGenerator NG>
Point3 CentroidCalculator<NG>::sample_polygon() {
    while (true) {
        Point3 sample_point = sample_bounding_box();

        if (_spherical_polygon.contains(sample_point)) {
            return sample_point;
        }
    }
}

template<NoiseGenerator NG>
Point3 CentroidCalculator<NG>::sample_bounding_box() {
    return _spherical_bounding_box_sampler.sample(_bounding_box);
}

}

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_CENTROID_CALCULATOR_HPP_
