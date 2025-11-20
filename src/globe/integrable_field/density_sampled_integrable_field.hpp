#ifndef GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_DENSITY_SAMPLED_INTEGRABLE_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_DENSITY_SAMPLED_INTEGRABLE_FIELD_HPP_

#include "../scalar_field/scalar_field.hpp"
#include "../globe_generator/spherical_polygon.hpp"
#include "../globe_generator/spherical_bounding_box.hpp"
#include "../globe_generator/sample_point_generator/bounding_box_sample_point_generator.hpp"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_sphere.h>
#include <vector>
#include <random>
#include <cmath>
#include <iterator>
#include <algorithm>
#include <utility>

namespace globe {

template<
    ScalarField SF,
    typename PointGenerator = BoundingBoxSamplePointGenerator
>
class DensitySampledIntegrableField {
 public:
    DensitySampledIntegrableField(
        SF &scalar_field,
        size_t target_sample_count = 200'000,
        PointGenerator point_generator = PointGenerator(),
        double max_density = 100.0,
        std::mt19937::result_type random_seed = std::random_device{}()
    );

    [[nodiscard]] double integrate(const SphericalPolygon &polygon) const;
    [[nodiscard]] double integrate_entire_sphere() const;
    [[nodiscard]] size_t sample_count() const;

 private:
    using Kernel = CGAL::Simple_cartesian<double>;
    using CGALPoint = Kernel::Point_3;
    using SearchTraits = CGAL::Search_traits_3<Kernel>;
    using KdTree = CGAL::Kd_tree<SearchTraits>;
    using FuzzySphere = CGAL::Fuzzy_sphere<SearchTraits>;

    SF &_scalar_field;
    PointGenerator _point_generator;
    SphericalBoundingBox _global_bounding_box;
    std::vector<Point3> _samples;
    std::vector<CGALPoint> _cgal_points;
    KdTree _kdtree;
    double _weight_per_sample;
    double _max_density;
    size_t _sample_attempts;
    std::mt19937 _random_engine;

    void build_samples(size_t target_sample_count);
    [[nodiscard]] double search_radius(const SphericalPolygon &polygon) const;
};

template<ScalarField SF, typename PointGenerator>
DensitySampledIntegrableField<SF, PointGenerator>::DensitySampledIntegrableField(
    SF &scalar_field,
    size_t target_sample_count,
    PointGenerator point_generator,
    double max_density,
    std::mt19937::result_type random_seed
) :
    _scalar_field(scalar_field),
    _point_generator(std::move(point_generator)),
    _global_bounding_box(),
    _weight_per_sample(0.0),
    _max_density(max_density),
    _sample_attempts(0),
    _random_engine(random_seed) {
    build_samples(target_sample_count);
}

template<ScalarField SF, typename PointGenerator>
void DensitySampledIntegrableField<SF, PointGenerator>::build_samples(
    size_t target_sample_count
) {
    if (target_sample_count == 0) {
        return;
    }

    std::uniform_real_distribution<double> acceptance_distribution(0.0, 1.0);

    size_t attempts = 0;
    double safe_max_density = std::max(_max_density, 1e-6);

    while (_samples.size() < target_sample_count) {
        Point3 candidate = _point_generator.generate(_global_bounding_box);
        attempts++;

        double density = _scalar_field.value(candidate);
        double acceptance_threshold = density / safe_max_density;
        acceptance_threshold = std::clamp(acceptance_threshold, 0.0, 1.0);
        double acceptance = acceptance_distribution(_random_engine);

        if (acceptance <= acceptance_threshold) {
            _samples.push_back(candidate);
        }
    }

    _sample_attempts = attempts;
    _weight_per_sample = (4.0 * M_PI * safe_max_density) / static_cast<double>(_sample_attempts);

    _cgal_points.reserve(_samples.size());
    for (const auto &point : _samples) {
        _cgal_points.emplace_back(point.x(), point.y(), point.z());
    }

    _kdtree.insert(_cgal_points.begin(), _cgal_points.end());
    _kdtree.build();
}

template<ScalarField SF, typename PointGenerator>
double DensitySampledIntegrableField<SF, PointGenerator>::integrate(
    const SphericalPolygon &polygon
) const {
    if (_samples.empty()) {
        return 0.0;
    }

    double radius = search_radius(polygon);

    auto bbox = polygon.bounding_box();
    double theta_mid = (bbox.theta_interval().low() + bbox.theta_interval().high()) / 2.0;
    double z_mid = (bbox.z_interval().low() + bbox.z_interval().high()) / 2.0;
    double r_mid = std::sqrt(std::max(0.0, 1.0 - z_mid * z_mid));

    CGALPoint center(
        r_mid * std::cos(theta_mid),
        r_mid * std::sin(theta_mid),
        z_mid
    );

    FuzzySphere query(center, radius, 0.0);
    std::vector<CGALPoint> candidates;
    _kdtree.search(std::back_inserter(candidates), query);

    size_t hits = 0;
    for (const auto &candidate : candidates) {
        Point3 point(candidate.x(), candidate.y(), candidate.z());
        if (polygon.contains(point)) {
            hits++;
        }
    }

    return static_cast<double>(hits) * _weight_per_sample;
}

template<ScalarField SF, typename PointGenerator>
double DensitySampledIntegrableField<SF, PointGenerator>::search_radius(
    const SphericalPolygon &polygon
) const {
    auto bbox = polygon.bounding_box();
    auto z_interval = bbox.z_interval();
    auto theta_interval = bbox.theta_interval();

    double z_span = z_interval.measure();
    double r_low = std::sqrt(std::max(0.0, 1.0 - z_interval.low() * z_interval.low()));
    double r_high = std::sqrt(std::max(0.0, 1.0 - z_interval.high() * z_interval.high()));
    double r_max = std::max(r_low, r_high);
    double chord = 2.0 * r_max * std::sin(theta_interval.measure() / 2.0);
    double radius = std::sqrt(z_span * z_span + chord * chord) * 1.1;

    return std::max(radius, 0.05);
}

template<ScalarField SF, typename PointGenerator>
double DensitySampledIntegrableField<SF, PointGenerator>::integrate_entire_sphere() const {
    return static_cast<double>(_samples.size()) * _weight_per_sample;
}

template<ScalarField SF, typename PointGenerator>
size_t DensitySampledIntegrableField<SF, PointGenerator>::sample_count() const {
    return _samples.size();
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_DENSITY_SAMPLED_INTEGRABLE_FIELD_HPP_

