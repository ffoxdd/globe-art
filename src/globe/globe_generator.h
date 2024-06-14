#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_

#include "types.h"
#include "sphere_mesh_generator/sphere_mesh_generator.h"
#include "point_generator/point_generator.h"
#include "point_generator/random_sphere_point_generator.h"
#include "points_collection/points_collection.h"
#include "noise_generator/noise_generator.h"
#include "noise_generator/anl_noise_generator.h"
#include <memory>

namespace globe {

const double LOW_DISTANCE_MULTIPLIER = 1.0 / 20.0;
const double HIGH_DISTANCE_MULTIPLIER = 1.0 / 4.0;
const int RANDOM_POINT_ITERATIONS = 10000;

template<
    PointGenerator PG = RandomSpherePointGenerator,
    NoiseGenerator NG = AnlNoiseGenerator<PG>
>
class GlobeGenerator {
 public:
    struct Config {
        double radius = 1.0;

        std::unique_ptr<PG> point_generator =
            std::make_unique<PG>(RandomSpherePointGenerator(1.0));

        std::unique_ptr<PointsCollection> points_collection =
            std::make_unique<PointsCollection>();

        std::unique_ptr<NG> noise_generator =
            std::make_unique<NG>(
                AnlNoiseGenerator<PG>(
                    AnlNoiseGenerator(AnlNoiseGenerator<>::Config{
                            .low = radius * LOW_DISTANCE_MULTIPLIER,
                            .high = radius * HIGH_DISTANCE_MULTIPLIER,
                            .point_generator = std::make_unique<PG>(RandomSpherePointGenerator(radius))
                        }
                    )
                )
            );
    };

    GlobeGenerator() : GlobeGenerator(Config()) { };

    explicit GlobeGenerator(Config &&config) :
        _point_generator(std::move(config.point_generator)),
        _points_collection(std::move(config.points_collection)),
        _noise_generator(std::move(config.noise_generator)) {
    };

    GlobeGenerator &generate_points();
    void save_ply(const std::string &filename) const;

    std::unique_ptr<PointsCollection> points_collection() {
        return std::move(_points_collection);
    }

 private:
    std::unique_ptr<PG> _point_generator;
    std::unique_ptr<PointsCollection> _points_collection;
    std::unique_ptr<NG> _noise_generator;
    std::vector<Point3> _points;
    // TODO: make center a data member

    void add_random_point();
    [[nodiscard]] bool too_close(const Point3 &point) const;
    [[nodiscard]] SurfaceMesh triangulation_mesh() const;
    static void save_mesh_ply(SurfaceMesh &mesh, const std::string &filename);
};

template<PointGenerator PG, NoiseGenerator NG>
GlobeGenerator<PG, NG>
&GlobeGenerator<PG, NG>::generate_points() {
    const int iterations = RANDOM_POINT_ITERATIONS;

    for (int i = 0; i < iterations; i++) {
        add_random_point();
    }

    return *this;
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeGenerator<PG, NG>::save_ply(const std::string &filename) const {
    SurfaceMesh mesh = triangulation_mesh();
    save_mesh_ply(mesh, filename);
}

template<PointGenerator PG, NoiseGenerator NG>
SurfaceMesh GlobeGenerator<PG, NG>::triangulation_mesh() const {
    SurfaceMesh mesh;
    std::map < Point3, SurfaceMesh::Vertex_index > vertices_by_point;

    for (auto const point : _points_collection->points()) {
        SurfaceMesh::Vertex_index vertex = mesh.add_vertex(point);
        vertices_by_point[point] = vertex;
    }

    for (auto const face : _points_collection->faces()) {
        SurfaceMesh::Vertex_index vertex0 = vertices_by_point[face->vertex(0)->point()];
        SurfaceMesh::Vertex_index vertex1 = vertices_by_point[face->vertex(1)->point()];
        SurfaceMesh::Vertex_index vertex2 = vertices_by_point[face->vertex(2)->point()];

        mesh.add_face(vertex0, vertex1, vertex2);
    }

    return std::move(mesh);
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeGenerator<PG, NG>::save_mesh_ply(
    SurfaceMesh &mesh, const std::string &filename
) {
    std::ofstream stream(filename);

    if (!stream) {
        throw std::runtime_error(std::format("Cannot open file for writing: {}", filename));
    }

    bool success = CGAL::IO::write_PLY(stream, mesh);

    if (!success) {
        throw std::runtime_error(std::format("Cannot write file: {}", filename));
    }
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeGenerator<PG, NG>::add_random_point() {
    Point3 point = _point_generator->generate();

    if (too_close(point)) {
        return;
    }

    _points_collection->insert(point);
}

template<PointGenerator PG, NoiseGenerator NG>
bool GlobeGenerator<PG, NG>::too_close(const Point3 &point) const {
    double separation_radius = _noise_generator->value(point);
    return !_points_collection->nearby_points(point, separation_radius).empty();
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
