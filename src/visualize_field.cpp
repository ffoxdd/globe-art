#include "globe/fields/spherical/harmonic_field.hpp"
#include "globe/generators/spherical/fibonacci_point_generator.hpp"
#include "globe/math/interval.hpp"
#include "globe/cgal/types.hpp"
#include "globe/io/qt/application.hpp"
#include "globe/io/qt/viewer.hpp"
#include <CGAL/convex_hull_3.h>
#include <CGAL/Surface_mesh.h>
#include <cmath>
#include <iostream>
#include <map>

using namespace globe;
using fields::spherical::HarmonicField;
using generators::spherical::FibonacciPointGenerator;
using io::qt::Application;
using io::qt::Viewer;
using io::qt::Color;

using SurfaceMesh = CGAL::Surface_mesh<cgal::Point3>;

void add_sphere_mesh(Viewer& viewer, const HarmonicField& field, size_t point_count);
Color value_to_color(double value, const Interval& range);

int main(int argc, char* argv[]) {
    int seed = 42;
    size_t point_count = 2500;

    if (argc > 1) {
        seed = std::atoi(argv[1]);
    }
    if (argc > 2) {
        point_count = static_cast<size_t>(std::atoi(argv[2]));
    }

    std::cout << "Visualizing HarmonicField with seed=" << seed <<
        ", points=" << point_count << std::endl;

    HarmonicField field(seed);

    Application app(argc, argv);
    Viewer viewer(nullptr, "Harmonic Field Visualization");

    add_sphere_mesh(viewer, field, point_count);

    viewer.center_on_unit_sphere();
    viewer.show();

    return app.run();
}

void add_sphere_mesh(Viewer& viewer, const HarmonicField& field, size_t point_count) {
    FibonacciPointGenerator generator;
    auto sphere_points = generator.generate(point_count);

    std::vector<cgal::Point3> cgal_points;
    cgal_points.reserve(point_count);
    for (const auto& p : sphere_points) {
        cgal_points.emplace_back(p.x(), p.y(), p.z());
    }

    SurfaceMesh mesh;
    CGAL::convex_hull_3(cgal_points.begin(), cgal_points.end(), mesh);

    std::cout << "Mesh: " << mesh.number_of_vertices() << " vertices, " <<
        mesh.number_of_faces() << " faces" << std::endl;

    std::map<cgal::Point3, double> point_values;
    Interval value_range;

    for (const auto& p : cgal_points) {
        VectorS2 v(p.x(), p.y(), p.z());
        double value = field.value(v);
        point_values[p] = value;
        value_range = Interval::hull(value_range, value);
    }

    std::cout << "Field range: [" << value_range.low() << ", " << value_range.high() << "]" << std::endl;

    for (auto face : mesh.faces()) {
        std::vector<VectorS2> vertices;
        double avg_value = 0.0;
        int count = 0;

        for (auto vertex : mesh.vertices_around_face(mesh.halfedge(face))) {
            cgal::Point3 p = mesh.point(vertex);
            vertices.emplace_back(p.x(), p.y(), p.z());
            avg_value += point_values[p];
            ++count;
        }

        avg_value /= count;
        Color color = value_to_color(avg_value, value_range);

        if (vertices.size() == 3) {
            viewer.add_triangle(vertices[0], vertices[1], vertices[2], color);
        }
    }
}

Color value_to_color(double value, const Interval& range) {
    constexpr Interval INTENSITY_RANGE(0.0, 255.0);
    int intensity = static_cast<int>(Interval::remap(value, range, INTENSITY_RANGE));
    return Color(intensity, intensity, intensity);
}
