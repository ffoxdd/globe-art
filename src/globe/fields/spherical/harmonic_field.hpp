#ifndef GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_HARMONIC_FIELD_HPP_
#define GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_HARMONIC_FIELD_HPP_

#include "field.hpp"
#include "../../types.hpp"
#include "../../geometry/spherical/arc.hpp"
#include "../../geometry/spherical/polygon/polygon.hpp"
#include <Eigen/Core>
#include <random>
#include <cmath>

namespace globe::fields::spherical {

using geometry::spherical::UNIT_SPHERE_AREA;

class HarmonicField {
 public:
    explicit HarmonicField(int seed = 42, double contrast = 0.5);

    [[nodiscard]] double value(const VectorS2& point) const;
    [[nodiscard]] double mass(const Polygon& polygon) const;
    [[nodiscard]] double total_mass() const;
    [[nodiscard]] double edge_integral(const Arc& arc) const;
    [[nodiscard]] Eigen::Vector3d edge_gradient_integral(const Arc& arc) const;

 private:
    double _constant;
    Eigen::Vector3d _linear;
    Eigen::Matrix3d _quadratic;
    double _total_mass;

    void initialize_coefficients(int seed, double contrast);
    void normalize_to_positive();

    [[nodiscard]] double evaluate_quadratic(const VectorS2& point) const;
    [[nodiscard]] double integrate_quadratic_polygon(const Polygon& polygon) const;
    [[nodiscard]] double integrate_quadratic_arc(const Arc& arc) const;
    [[nodiscard]] Eigen::Vector3d gradient_integrate_quadratic_arc(const Arc& arc) const;
};

inline HarmonicField::HarmonicField(int seed, double contrast) {
    initialize_coefficients(seed, contrast);
    normalize_to_positive();
}

inline void HarmonicField::initialize_coefficients(int seed, double contrast) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> dist(0.0, 1.0);

    _linear = Eigen::Vector3d(dist(rng), dist(rng), dist(rng));

    _quadratic = Eigen::Matrix3d::Zero();
    _quadratic(0, 0) = dist(rng);
    _quadratic(1, 1) = dist(rng);
    _quadratic(2, 2) = -_quadratic(0, 0) - _quadratic(1, 1);

    double xy = dist(rng);
    double xz = dist(rng);
    double yz = dist(rng);
    _quadratic(0, 1) = _quadratic(1, 0) = xy;
    _quadratic(0, 2) = _quadratic(2, 0) = xz;
    _quadratic(1, 2) = _quadratic(2, 1) = yz;

    _linear *= contrast;
    _quadratic *= contrast;

    _constant = 1.0;
}

inline void HarmonicField::normalize_to_positive() {
    constexpr int SAMPLES = 1000;
    constexpr double GOLDEN_RATIO = 1.618033988749895;
    constexpr double MIN_VALUE = 0.1;

    double min_found = std::numeric_limits<double>::max();

    for (int i = 0; i < SAMPLES; ++i) {
        double theta = 2.0 * M_PI * i / GOLDEN_RATIO;
        double z = 1.0 - 2.0 * (i + 0.5) / SAMPLES;
        double r = std::sqrt(1.0 - z * z);
        VectorS2 point(r * std::cos(theta), r * std::sin(theta), z);

        double val = value(point);
        min_found = std::min(min_found, val);
    }

    if (min_found < MIN_VALUE) {
        _constant += (MIN_VALUE - min_found);
    }

    _total_mass = _constant * UNIT_SPHERE_AREA;
}

inline double HarmonicField::value(const VectorS2& point) const {
    return _constant + _linear.dot(point) + evaluate_quadratic(point);
}

inline double HarmonicField::evaluate_quadratic(const VectorS2& point) const {
    return point.transpose() * _quadratic * point;
}

inline double HarmonicField::mass(const Polygon& polygon) const {
    double constant_term = _constant * polygon.area();
    double linear_term = _linear.dot(polygon.first_moment());
    double quadratic_term = integrate_quadratic_polygon(polygon);

    return constant_term + linear_term + quadratic_term;
}

inline double HarmonicField::integrate_quadratic_polygon(const Polygon& polygon) const {
    Eigen::Matrix3d moment = polygon.second_moment();
    double result = 0.0;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result += _quadratic(i, j) * moment(i, j);
        }
    }

    return result;
}

inline double HarmonicField::total_mass() const {
    return _total_mass;
}

inline double HarmonicField::edge_integral(const Arc& arc) const {
    double constant_term = _constant * arc.length();
    double linear_term = _linear.dot(arc.first_moment());
    double quadratic_term = integrate_quadratic_arc(arc);

    return constant_term + linear_term + quadratic_term;
}

inline double HarmonicField::integrate_quadratic_arc(const Arc& arc) const {
    Eigen::Matrix3d moment = arc.second_moment();
    double result = 0.0;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result += _quadratic(i, j) * moment(i, j);
        }
    }

    return result;
}

inline Eigen::Vector3d HarmonicField::edge_gradient_integral(const Arc& arc) const {
    Eigen::Vector3d constant_term = _constant * arc.first_moment();
    Eigen::Vector3d linear_term = arc.second_moment() * _linear;
    Eigen::Vector3d quadratic_term = gradient_integrate_quadratic_arc(arc);

    return constant_term + linear_term + quadratic_term;
}

inline Eigen::Vector3d HarmonicField::gradient_integrate_quadratic_arc(const Arc& arc) const {
    constexpr size_t SAMPLES = 50;
    double arc_length = arc.length();

    if (arc_length < 1e-10) {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < SAMPLES; ++i) {
        double t = (i + 0.5) / SAMPLES;
        VectorS2 point = arc.interpolate(t);
        double quad_value = evaluate_quadratic(point);
        sum += quad_value * point;
    }

    return sum * arc_length / SAMPLES;
}

static_assert(Field<HarmonicField>);

} // namespace globe::fields::spherical

#endif //GLOBEART_SRC_GLOBE_FIELDS_SPHERICAL_HARMONIC_FIELD_HPP_
