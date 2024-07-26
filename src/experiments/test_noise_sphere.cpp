#include "../globe/geometry_viewer/geometry_viewer.hpp"
#include "../globe/sphere_mesh_generator/sphere_mesh_generator.hpp"
#include "../globe/noise_generator/anl_noise_generator.hpp"
#include <QtWidgets/QApplication>
#include <CGAL/Qt/init_ogl_context.h>

using namespace globe;

int main(int argc, char *argv[]) {
    CGAL::Qt::init_ogl_context(4, 3);
    QApplication app(argc, argv);

    GeometryViewer geometry_viewer(QApplication::activeWindow());
    SphereMeshGenerator sphere_mesh_generator;
    AnlNoiseGenerator noise_generator;

    auto sphere_mesh = SphereMeshGenerator::generate(1.0, 6, Point3(0, 0, 0));

    auto point_range = sphere_mesh.vertices() | std::views::transform(
        [&sphere_mesh](auto v) { return sphere_mesh.point(v); }
    );

    auto sample_points = std::vector<Point3>(point_range.begin(), point_range.end());

    noise_generator.normalize(sample_points, Interval(0, 255));

    for (const auto face : sphere_mesh.faces()) {
        auto face_vertex_range =
            CGAL::vertices_around_face(sphere_mesh.halfedge(face), sphere_mesh) | std::views::transform(
                [&sphere_mesh](auto v) { return sphere_mesh.point(v); }
            );

        auto face_points = std::vector<Point3>(face_vertex_range.begin(), face_vertex_range.end());

        double value = 0.0;
        for (auto point : face_points) {
            value += noise_generator.value(point);
        }
        value /= static_cast<double>(face_points.size());
        auto int_value = static_cast<int>(value);

        CGAL::IO::Color color;

        if (int_value > 128) {
            color = CGAL::IO::Color(255, 255, 255);
        } else {
            color = CGAL::IO::Color(0, 0, 0);
        }

//        auto color = CGAL::IO::Color(int_value, int_value, int_value);

        geometry_viewer.face_begin(color);

        for (auto point : face_points) {
            geometry_viewer.add_point_in_face(point);
        }

        geometry_viewer.face_end();
    }

    geometry_viewer.show();
    return QApplication::exec();
}
