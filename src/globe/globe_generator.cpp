#include "globe_generator.h"
#include <iostream>
#include <CGAL/Polygon_mesh_processing/corefinement.h>

namespace globe {

GlobeGenerator &GlobeGenerator::generate() {
    const int iterations = 10000;

    for (int i = 0; i < iterations; i++) {
        add_random_point();
    }

    return *this;
}

void GlobeGenerator::save_ply(const std::string &filename) const {
    SurfaceMesh mesh = render();
    save_mesh_ply(mesh, filename);
}

SurfaceMesh GlobeGenerator::render() const {
//    return render_points();
    return render_triangulation();
}

SurfaceMesh GlobeGenerator::render_triangulation() const {
    SurfaceMesh mesh;
    std::map<Point3, SurfaceMesh::Vertex_index> vertices_by_point;

    for (auto const point : _points_collection->points()) {
        SurfaceMesh::Vertex_index vertex = mesh.add_vertex(point);
        vertices_by_point[point] = vertex;
    }

    for (auto const face : _points_collection->faces()) {
        SurfaceMesh::Vertex_index vertex_0 = vertices_by_point[face.vertex(0)->point()];
        SurfaceMesh::Vertex_index vertex_1 = vertices_by_point[face.vertex(1)->point()];
        SurfaceMesh::Vertex_index vertex_2 = vertices_by_point[face.vertex(2)->point()];

        mesh.add_face(vertex_0, vertex_1, vertex_2);
    }

    return std::move(mesh);
}

SurfaceMesh GlobeGenerator::render_points() const {
    SurfaceMesh mesh = generate_globe_sphere();
    return std::move(add_points_to_mesh(mesh));
}

void GlobeGenerator::save_mesh_ply(SurfaceMesh &mesh, const std::string &filename) {
    std::ofstream stream(filename);

    if (!stream) {
        throw std::runtime_error(std::format("Cannot open file for writing: {}", filename));
    }

    bool success = CGAL::IO::write_PLY(stream, mesh);

    if (!success) {
        throw std::runtime_error(std::format("Cannot write file: {}", filename));
    }
}

SurfaceMesh GlobeGenerator::generate_globe_sphere() const {
    return _sphere_mesh_generator->generate(_radius, 5, Point3(0, 0, 0));
}

void GlobeGenerator::add_random_point() {
    Point3 point = _random_point_generator->generate();

    if (too_close(point)) {
        return;
    }

    _points_collection->insert(point);
}

bool GlobeGenerator::too_close(const Point3 &point) const {
    double separation_radius = _noise_generator->value(point);
    return !_points_collection->nearby_points(point, separation_radius).empty();
}

// TODO: Factor out a mesh builder class to facilitate incremental mesh building

SurfaceMesh GlobeGenerator::add_points_to_mesh(SurfaceMesh &mesh) const {
    for (const Point3 &point : _points_collection->points()) {
        mesh = add_point_to_mesh(mesh, point);
    }

    return mesh;
}

SurfaceMesh GlobeGenerator::add_point_to_mesh(SurfaceMesh &mesh, const Point3 &point) const {
    const double point_mesh_radius = _radius / 50;
    SurfaceMesh point_mesh = _sphere_mesh_generator->generate(point_mesh_radius, 1, point);
    return add_meshes(mesh, point_mesh);
}

SurfaceMesh GlobeGenerator::add_meshes(SurfaceMesh &mesh_1, SurfaceMesh &mesh_2) {
    SurfaceMesh result;
    CGAL::Polygon_mesh_processing::corefine_and_compute_union(mesh_1, mesh_2, result);
    return std::move(result);
}

} // namespace globe
