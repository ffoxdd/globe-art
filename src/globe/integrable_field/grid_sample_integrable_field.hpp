#ifndef GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_GRID_SAMPLE_INTEGRABLE_FIELD_H_
#define GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_GRID_SAMPLE_INTEGRABLE_FIELD_H_

#include "integrable_field.hpp"
#include "../scalar_field/scalar_field.hpp"
#include "../globe_generator/spherical_polygon.hpp"
#include "../types.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_sphere.h>

namespace globe {

struct CGALPointHash {
    std::size_t operator()(const CGAL::Simple_cartesian<double>::Point_3 &p) const {
        std::size_t h1 = std::hash<double>{}(p.x());
        std::size_t h2 = std::hash<double>{}(p.y());
        std::size_t h3 = std::hash<double>{}(p.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

template<ScalarField SF>
class GridSampleIntegrableField {
 public:
    explicit GridSampleIntegrableField(SF scalar_field, size_t num_samples = 100000);
    double integrate(const SphericalPolygon &polygon);
    void print_stats() const;

 private:
    using Kernel = CGAL::Simple_cartesian<double>;
    using CGALPoint = Kernel::Point_3;
    using SearchTraits = CGAL::Search_traits_3<Kernel>;
    using Splitter = CGAL::Sliding_midpoint<SearchTraits>;
    using KdTree = CGAL::Kd_tree<SearchTraits, Splitter>;

    SF _scalar_field;
    std::vector<Point3> _sample_points;
    std::vector<double> _sample_densities;
    std::vector<CGALPoint> _cgal_points;
    std::unordered_map<CGALPoint, size_t, CGALPointHash> _point_to_index;
    double _area_per_sample;
    KdTree _kdtree;

    mutable size_t _total_integrations;
    mutable size_t _total_samples_tested;
    mutable size_t _total_candidates_from_kdtree;

    void generate_fibonacci_sphere(size_t n);
};

template<ScalarField SF>
GridSampleIntegrableField<SF>::GridSampleIntegrableField(
    SF scalar_field,
    size_t num_samples
) :
    _scalar_field(scalar_field),
    _total_integrations(0),
    _total_samples_tested(0),
    _total_candidates_from_kdtree(0) {

    std::cout << std::endl;
    std::cout << "Generating Fibonacci sphere with " << num_samples << " samples..." << std::flush;
    generate_fibonacci_sphere(num_samples);
    std::cout << " done" << std::endl;

    std::cout << "Pre-computing densities and building KD-tree..." << std::flush;
    _sample_densities.reserve(num_samples);
    _cgal_points.reserve(num_samples);
    for (size_t i = 0; i < _sample_points.size(); i++) {
        const auto &point = _sample_points[i];
        _sample_densities.push_back(_scalar_field.value(point));
        CGALPoint cgal_point(point.x(), point.y(), point.z());
        _cgal_points.push_back(cgal_point);
        _point_to_index[cgal_point] = i;
    }
    _kdtree.insert(_cgal_points.begin(), _cgal_points.end());
    _kdtree.build();
    std::cout << " done" << std::endl;

    _area_per_sample = (4.0 * M_PI) / num_samples;
    std::cout << "Grid initialized: " << num_samples << " samples, " <<
        _area_per_sample << " area per sample" << std::endl;
}

template<ScalarField SF>
void GridSampleIntegrableField<SF>::generate_fibonacci_sphere(size_t n) {
    _sample_points.reserve(n);

    const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    const double golden_angle = 2.0 * M_PI / (phi + 1.0);

    for (size_t i = 0; i < n; i++) {
        double z = 1.0 - (2.0 * i + 1.0) / n;
        double radius = std::sqrt(1.0 - z * z);
        double theta = golden_angle * i;

        double x = radius * std::cos(theta);
        double y = radius * std::sin(theta);

        _sample_points.emplace_back(x, y, z);
    }
}

template<ScalarField SF>
double GridSampleIntegrableField<SF>::integrate(const SphericalPolygon &polygon) {
    _total_integrations++;

    auto bbox = polygon.bounding_box();
    auto z_interval = bbox.z_interval();
    auto theta_interval = bbox.theta_interval();

    double z_center = (z_interval.low() + z_interval.high()) / 2.0;
    double r_center = std::sqrt(1.0 - z_center * z_center);
    double theta_center = (theta_interval.low() + theta_interval.high()) / 2.0;

    Point3 center(
        r_center * std::cos(theta_center),
        r_center * std::sin(theta_center),
        z_center
    );

    double z_span = z_interval.measure();
    double r_at_low_z = std::sqrt(1.0 - z_interval.low() * z_interval.low());
    double r_at_high_z = std::sqrt(1.0 - z_interval.high() * z_interval.high());
    double r_max = std::max(r_at_low_z, r_at_high_z);
    double chord_length = 2.0 * r_max * std::sin(theta_interval.measure() / 2.0);

    double search_radius = std::sqrt(
        z_span * z_span + chord_length * chord_length
    ) * 0.55;

    using FuzzySphere = CGAL::Fuzzy_sphere<SearchTraits>;
    FuzzySphere query_sphere(
        CGALPoint(center.x(), center.y(), center.z()),
        search_radius,
        0.0
    );

    std::vector<CGALPoint> candidates;
    _kdtree.search(std::back_inserter(candidates), query_sphere);
    _total_candidates_from_kdtree += candidates.size();

    double sum = 0.0;
    for (const auto &cgal_point : candidates) {
        _total_samples_tested++;
        Point3 point(cgal_point.x(), cgal_point.y(), cgal_point.z());

        if (polygon.contains(point)) {
            auto it = _point_to_index.find(cgal_point);
            if (it != _point_to_index.end()) {
                sum += _sample_densities[it->second];
            }
        }
    }

    return sum * _area_per_sample;
}

template<ScalarField SF>
void GridSampleIntegrableField<SF>::print_stats() const {
    std::cout << "\nGrid sample integration statistics:" << std::endl;
    std::cout << "  Total integrations: " << _total_integrations << std::endl;
    std::cout << "  Total candidates from KD-tree: " << _total_candidates_from_kdtree << std::endl;
    std::cout << "  Average candidates per integration: " <<
        (_total_integrations > 0 ? _total_candidates_from_kdtree / _total_integrations : 0) <<
        std::endl;
    std::cout << "  Total samples tested: " << _total_samples_tested << std::endl;
    std::cout << "  Average samples tested per integration: " <<
        (_total_integrations > 0 ? _total_samples_tested / _total_integrations : 0) <<
        std::endl;
}

}

#endif //GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_GRID_SAMPLE_INTEGRABLE_FIELD_H_
