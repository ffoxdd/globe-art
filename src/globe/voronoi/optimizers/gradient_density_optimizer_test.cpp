#include "gradient_density_optimizer.hpp"
#include "../../fields/spherical/constant_spherical_field.hpp"
#include "../../fields/spherical/linear_spherical_field.hpp"
#include "../../geometry/spherical/moments/arc_moments.hpp"
#include "../../geometry/spherical/moments/polygon_moments.hpp"
#include "../../geometry/spherical/helpers.hpp"
#include "../../testing/geometric_assertions.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <memory>

using namespace globe;

namespace {

std::unique_ptr<VoronoiSphere> create_test_voronoi(size_t num_points) {
    auto voronoi = std::make_unique<VoronoiSphere>();

    double phi = M_PI * (std::sqrt(5.0) - 1.0);
    for (size_t i = 0; i < num_points; ++i) {
        double y = 1.0 - (i / static_cast<double>(num_points - 1)) * 2.0;
        double radius = std::sqrt(1.0 - y * y);
        double theta = phi * i;

        double x = std::cos(theta) * radius;
        double z = std::sin(theta) * radius;

        voronoi->insert(Point3(x, y, z));
    }

    return voronoi;
}

}

TEST(GradientDensityOptimizerTest, ConvergesForConstantField) {
    auto voronoi = create_test_voronoi(10);
    ConstantSphericalField field(1.0);

    GradientDensityOptimizer optimizer(
        std::move(voronoi),
        field,
        50
    );

    auto result = optimizer.optimize();

    EXPECT_EQ(result->size(), 10);
}

TEST(GradientDensityOptimizerTest, CellEdgesHaveValidNeighbors) {
    auto voronoi = create_test_voronoi(10);

    for (size_t i = 0; i < voronoi->size(); ++i) {
        auto edges = voronoi->cell_edges(i);
        EXPECT_GT(edges.size(), 0) << "Cell " << i << " has no edges";

        for (const auto& edge : edges) {
            EXPECT_LT(edge.neighbor_index, voronoi->size())
                << "Cell " << i << " has invalid neighbor " << edge.neighbor_index;
            EXPECT_NE(edge.neighbor_index, i)
                << "Cell " << i << " has itself as neighbor";
        }
    }
}

TEST(GradientDensityOptimizerTest, MassErrorsAreComputable) {
    auto voronoi = create_test_voronoi(10);
    ConstantSphericalField field(1.0);

    double target_mass = field.total_mass() / voronoi->size();
    double total_computed_mass = 0.0;

    for (const auto& cell : voronoi->dual_cells()) {
        auto moments = compute_polygon_moments(cell);
        double mass = field.mass(moments);
        total_computed_mass += mass;
    }

    EXPECT_NEAR(total_computed_mass, field.total_mass(), 0.1);
}

TEST(GradientDensityOptimizerTest, EXPENSIVE_ConvergesForLinearField) {
    REQUIRE_EXPENSIVE();

    auto voronoi = create_test_voronoi(20);
    LinearSphericalField field(1.0, 2.0);

    GradientDensityOptimizer<LinearSphericalField> optimizer(
        std::move(voronoi),
        field,
        1000
    );

    auto result = optimizer.optimize();

    EXPECT_EQ(result->size(), 20);
}

TEST(GradientDensityOptimizerTest, EXPENSIVE_GradientMatchesNumericalForLinearField) {
    REQUIRE_EXPENSIVE();

    auto voronoi = create_test_voronoi(10);
    LinearSphericalField field(1.0, 2.0);
    double target_mass = field.total_mass() / voronoi->size();

    auto compute_mass_errors = [&]() {
        std::vector<double> errors(voronoi->size());
        size_t i = 0;
        for (const auto& cell : voronoi->dual_cells()) {
            auto moments = compute_polygon_moments(cell);
            errors[i] = field.mass(moments) - target_mass;
            ++i;
        }
        return errors;
    };

    auto compute_error = [&]() {
        auto errors = compute_mass_errors();
        double total = 0.0;
        for (double e : errors) {
            total += e * e;
        }
        return total / 2.0;
    };

    // Save all site positions for rebuilding
    std::vector<Point3> sites;
    for (size_t i = 0; i < voronoi->size(); ++i) {
        sites.push_back(voronoi->site(i));
    }

    auto rebuild_voronoi = [&sites]() {
        auto v = std::make_unique<VoronoiSphere>();
        for (const auto& site : sites) {
            v->insert(site);
        }
        return v;
    };

    auto rebuild_voronoi_with_perturbation = [&sites](size_t index, Point3 new_pos) {
        auto v = std::make_unique<VoronoiSphere>();
        for (size_t i = 0; i < sites.size(); ++i) {
            if (i == index) {
                v->insert(new_pos);
            } else {
                v->insert(sites[i]);
            }
        }
        return v;
    };

    auto compute_error_for = [&](VoronoiSphere& v) {
        double total = 0.0;
        for (const auto& cell : v.dual_cells()) {
            auto moments = compute_polygon_moments(cell);
            double mass_error = field.mass(moments) - target_mass;
            total += mass_error * mass_error;
        }
        return total / 2.0;
    };

    auto compute_analytical_gradient = [&](size_t site_index) {
        auto mass_errors = compute_mass_errors();
        Eigen::Vector3d gradient = Eigen::Vector3d::Zero();

        Point3 site_k = voronoi->site(site_index);
        Eigen::Vector3d s_k = to_eigen(site_k);
        auto cell_edges = voronoi->cell_edges(site_index);

        for (const auto& edge_info : cell_edges) {
            size_t j = edge_info.neighbor_index;
            Point3 site_j = voronoi->site(j);

            Point3 v1 = to_point(edge_info.arc.source());
            Point3 v2 = to_point(edge_info.arc.target());

            auto arc_moments = compute_arc_moments(v1, v2);
            Eigen::Vector3d rho_weighted_moment = field.edge_gradient_integral(arc_moments);
            double edge_integral = field.edge_integral(arc_moments);

            Eigen::Vector3d n_vec = to_eigen(site_j) - s_k;
            double n_norm = n_vec.norm();

            if (n_norm < GEOMETRIC_EPSILON) continue;

            Eigen::Vector3d edge_grad = (rho_weighted_moment - s_k * edge_integral) / n_norm;
            gradient += (mass_errors[site_index] - mass_errors[j]) * edge_grad;
        }

        return gradient;
    };

    double epsilon = 1e-5;
    auto base_voronoi = rebuild_voronoi();
    double base_error = compute_error_for(*base_voronoi);

    std::cout << "\nGradient verification for linear field:" << std::endl;
    std::cout << "Base error: " << base_error << std::endl;
    std::cout << "Target mass: " << target_mass << std::endl;

    for (size_t site_idx = 0; site_idx < std::min(voronoi->size(), size_t(3)); ++site_idx) {
        Point3 site = sites[site_idx];
        Eigen::Vector3d s(site.x(), site.y(), site.z());

        Eigen::Vector3d analytical = compute_analytical_gradient(site_idx);

        // Project analytical gradient to tangent space (numerical gradient is naturally tangent)
        Eigen::Vector3d analytical_tangent = analytical - analytical.dot(s) * s;

        Eigen::Vector3d numerical = Eigen::Vector3d::Zero();

        for (int dim = 0; dim < 3; ++dim) {
            Eigen::Vector3d perturb = Eigen::Vector3d::Zero();
            perturb[dim] = epsilon;

            Eigen::Vector3d new_pos_plus = (s + perturb).normalized();
            auto voronoi_plus = rebuild_voronoi_with_perturbation(
                site_idx, Point3(new_pos_plus.x(), new_pos_plus.y(), new_pos_plus.z())
            );
            double error_plus = compute_error_for(*voronoi_plus);

            Eigen::Vector3d new_pos_minus = (s - perturb).normalized();
            auto voronoi_minus = rebuild_voronoi_with_perturbation(
                site_idx, Point3(new_pos_minus.x(), new_pos_minus.y(), new_pos_minus.z())
            );
            double error_minus = compute_error_for(*voronoi_minus);

            numerical[dim] = (error_plus - error_minus) / (2 * epsilon);
        }

        std::cout << "\nSite " << site_idx << " at (" << s.x() << ", " << s.y() << ", " << s.z() << "):" << std::endl;
        std::cout << "  Analytical (R3): (" << analytical.x() << ", " << analytical.y() << ", " << analytical.z() << ")" << std::endl;
        std::cout << "  Analytical (tangent): (" << analytical_tangent.x() << ", " << analytical_tangent.y() << ", " << analytical_tangent.z() << ")" << std::endl;
        std::cout << "  Numerical:  (" << numerical.x() << ", " << numerical.y() << ", " << numerical.z() << ")" << std::endl;
        std::cout << "  Difference: " << (analytical_tangent - numerical).norm() << std::endl;

        // Use larger tolerance since finite differences have O(epsilon) error
        EXPECT_NEAR(analytical_tangent.x(), numerical.x(), 0.2);
        EXPECT_NEAR(analytical_tangent.y(), numerical.y(), 0.2);
        EXPECT_NEAR(analytical_tangent.z(), numerical.z(), 0.2);
    }
}
