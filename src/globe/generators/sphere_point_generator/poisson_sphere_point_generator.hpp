#ifndef GLOBEART_SRC_GLOBE_GENERATORS_POISSON_SPHERE_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GENERATORS_POISSON_SPHERE_POINT_GENERATOR_HPP_

#include "../../types.hpp"
#include "../../geometry/spherical/spherical_bounding_box.hpp"
#include "../../fields/scalar/scalar_field.hpp"
#include "../../fields/scalar/constant_scalar_field.hpp"
#include "sphere_point_generator.hpp"
#include "random_sphere_point_generator.hpp"
#include <vector>
#include <cmath>

namespace globe {

template<
    ScalarField DensityFieldType = ConstantScalarField,
    SpherePointGenerator SpherePointGeneratorType = RandomSpherePointGenerator<>
>
class PoissonSpherePointGenerator {
 public:
    PoissonSpherePointGenerator() = default;

    explicit PoissonSpherePointGenerator(
        DensityFieldType density_field,
        SpherePointGeneratorType generator = SpherePointGeneratorType()
    ) : _density_field(std::move(density_field)),
        _generator(std::move(generator)) {
    }

    std::vector<Point3> generate(
        size_t count,
        const SphericalBoundingBox &bounding_box = SphericalBoundingBox::full_sphere()
    );

 private:
    DensityFieldType _density_field;
    SpherePointGeneratorType _generator;

    static constexpr size_t MAX_ATTEMPTS_PER_POINT = 30;
    static constexpr double DISTANCE_SCALING_FACTOR = 0.95;

    double base_min_distance(size_t count, double area) const;
    double min_distance_at(const Point3 &point, double base_distance) const;

    std::vector<Point3> generate_poisson_samples(
        size_t target_count,
        double base_distance,
        const SphericalBoundingBox &bbox
    );

    bool is_valid_candidate(const Point3 &candidate, const KDTree &tree, double base_distance) const;
    void adjust_to_exact_count(std::vector<Point3> &points, size_t target_count);
};

template<ScalarField DensityFieldType, SpherePointGenerator SpherePointGeneratorType>
std::vector<Point3> PoissonSpherePointGenerator<DensityFieldType, SpherePointGeneratorType>::generate(
    size_t count,
    const SphericalBoundingBox &bounding_box
) {
    if (count == 0) {
        return {};
    }

    double area = bounding_box.area();
    double base_distance = base_min_distance(count, area);

    auto points = generate_poisson_samples(count, base_distance, bounding_box);
    adjust_to_exact_count(points, count);

    return points;
}

template<ScalarField DensityFieldType, SpherePointGenerator SpherePointGeneratorType>
double PoissonSpherePointGenerator<DensityFieldType, SpherePointGeneratorType>::base_min_distance(
    size_t count,
    double area
) const {
    return std::sqrt(area / (count * M_PI)) * DISTANCE_SCALING_FACTOR;
}

template<ScalarField DensityFieldType, SpherePointGenerator SpherePointGeneratorType>
double PoissonSpherePointGenerator<DensityFieldType, SpherePointGeneratorType>::min_distance_at(
    const Point3 &point,
    double base_distance
) const {
    double density = _density_field.value(point);
    if (density <= 0.0) {
        return std::numeric_limits<double>::max();
    }
    return base_distance / std::sqrt(density);
}

template<ScalarField DensityFieldType, SpherePointGenerator SpherePointGeneratorType>
std::vector<Point3> PoissonSpherePointGenerator<DensityFieldType, SpherePointGeneratorType>::generate_poisson_samples(
    size_t target_count,
    double base_distance,
    const SphericalBoundingBox &bbox
) {
    std::vector<Point3> points;
    points.reserve(target_count);

    KDTree tree;

    size_t attempts = 0;
    size_t max_total_attempts = target_count * MAX_ATTEMPTS_PER_POINT;

    while (points.size() < target_count && attempts < max_total_attempts) {
        auto candidates = _generator.generate(1, bbox);
        Point3 candidate = candidates[0];
        attempts++;

        if (is_valid_candidate(candidate, tree, base_distance)) {
            points.push_back(candidate);
            tree.insert(candidate);
        }
    }

    return points;
}

template<ScalarField DensityFieldType, SpherePointGenerator SpherePointGeneratorType>
bool PoissonSpherePointGenerator<DensityFieldType, SpherePointGeneratorType>::is_valid_candidate(
    const Point3 &candidate,
    const KDTree &tree,
    double base_distance
) const {
    if (tree.size() == 0) {
        return true;
    }

    double local_min_distance = min_distance_at(candidate, base_distance);

    FuzzySphere query(candidate, local_min_distance, 0.0);
    std::vector<Point3> neighbors;
    tree.search(std::back_inserter(neighbors), query);

    return neighbors.empty();
}

template<ScalarField DensityFieldType, SpherePointGenerator SpherePointGeneratorType>
void PoissonSpherePointGenerator<DensityFieldType, SpherePointGeneratorType>::adjust_to_exact_count(
    std::vector<Point3> &points,
    size_t target_count
) {
    if (points.size() > target_count) {
        points.resize(target_count);
    } else if (points.size() < target_count) {
        size_t needed = target_count - points.size();
        auto fill_points = _generator.generate(needed);
        points.insert(points.end(), fill_points.begin(), fill_points.end());
    }
}

}

#endif //GLOBEART_SRC_GLOBE_GENERATORS_POISSON_SPHERE_POINT_GENERATOR_HPP_
