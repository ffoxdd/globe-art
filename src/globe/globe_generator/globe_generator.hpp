#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_

#include "../types.hpp"
#include "../sphere_mesh_generator/sphere_mesh_generator.hpp"
#include "../point_generator/point_generator.hpp"
#include "../point_generator/random_sphere_point_generator.hpp"
#include "../points_collection/points_collection.hpp"
#include "../noise_generator/noise_generator.hpp"
#include "../noise_generator/anl_noise_generator.hpp"
#include <memory>

namespace globe {

const Interval POINT_DISTANCE_RANGE = Interval(1.0 / 20.0, 1.0 / 4.0);
const int RANDOM_POINT_ITERATIONS = 10000;

template<
    PointGenerator PG = RandomSpherePointGenerator,
    NoiseGenerator NG = AnlNoiseGenerator
>
class GlobeGenerator {
 public:
    struct Config;

    GlobeGenerator();
    explicit GlobeGenerator(Config &&config);

    GlobeGenerator &build();
    void save_ply(const std::string &filename) const;

    auto dual_arcs() -> decltype(auto);
    auto dual_neighborhoods() -> decltype(auto);

 private:
    std::unique_ptr<PG> _point_generator;
    std::unique_ptr<PointsCollection> _points_collection;
    std::unique_ptr<NG> _noise_generator;

    void normalize_noise();
    void add_points();
    void relax();

    void add_point();
    std::vector<Point3> sample_points(size_t n);
    [[nodiscard]] bool too_close(const Point3 &point) const;
    [[nodiscard]] SurfaceMesh triangulation_mesh() const;
    static void save_mesh_ply(SurfaceMesh &mesh, const std::string &filename);
};

template<PointGenerator PG, NoiseGenerator NG>
struct GlobeGenerator<PG, NG>::Config {
    double radius = 1.0;

    std::unique_ptr<PG> point_generator = std::make_unique<PG>(
        RandomSpherePointGenerator(RandomSpherePointGenerator::Config{.radius = radius})
    );

    std::unique_ptr<PointsCollection> points_collection = std::make_unique<PointsCollection>(
    );

    std::unique_ptr<NG> noise_generator = std::make_unique<NG>(
        AnlNoiseGenerator()
    );
};

template<PointGenerator PG, NoiseGenerator NG>
GlobeGenerator<PG, NG>::GlobeGenerator() : GlobeGenerator(Config()) { }

template<PointGenerator PG, NoiseGenerator NG>
GlobeGenerator<PG, NG>::GlobeGenerator(GlobeGenerator::Config &&config) :
    _point_generator(std::move(config.point_generator)),
    _points_collection(std::move(config.points_collection)),
    _noise_generator(std::move(config.noise_generator)) {
}

template<PointGenerator PG, NoiseGenerator NG>
GlobeGenerator<PG, NG> &GlobeGenerator<PG, NG>::build() {
    normalize_noise();
    add_points();
    // relax
    return *this;
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeGenerator<PG, NG>::save_ply(const std::string &filename) const {
    SurfaceMesh mesh = triangulation_mesh();
    save_mesh_ply(mesh, filename);
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeGenerator<PG, NG>::normalize_noise() {
    _noise_generator->normalize(sample_points(1000), POINT_DISTANCE_RANGE);
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeGenerator<PG, NG>::add_points() {
    const int iterations = RANDOM_POINT_ITERATIONS;

    for (int i = 0; i < iterations; i++) {
        add_point();
    }
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeGenerator<PG, NG>::relax() {
    for (const auto &dual_neighborhood : _points_collection->dual_neighborhoods()) {
        // make a spherical polygon
        // get its area and spherical bounding box
        // randomly generate points in the bounding box
        // if the point is in the polygon, add it to the centroid calculation
        // stop after some number determined by the area
        // store the result in a map from vertex point -> weighted centroid
        // at the end, move all vertex points to the weighted centroid

//        auto cell_points = dual_neighborhood.dual_cell_arcs | std::ranges::views::transform(
//            [](const auto &arc) { return arc->vertex(0)->point(); }
//        );
    }
}

template<PointGenerator PG, NoiseGenerator NG>
std::vector<Point3> GlobeGenerator<PG, NG>::sample_points(size_t n) {
    std::vector<Point3> points;

    for (size_t i = 0; i < n; i++) {
        points.push_back(_point_generator->generate());
    }

    return points;
}

template<PointGenerator PG, NoiseGenerator NG>
auto GlobeGenerator<PG, NG>::dual_arcs() -> decltype(auto) {
    return _points_collection->dual_arcs();
}

template<PointGenerator PG, NoiseGenerator NG>
auto GlobeGenerator<PG, NG>::dual_neighborhoods() -> decltype(auto) {
    return _points_collection->dual_neighborhoods();
}

template<PointGenerator PG, NoiseGenerator NG>
SurfaceMesh GlobeGenerator<PG, NG>::triangulation_mesh() const {
    SurfaceMesh mesh;
    std::map < Point3, SurfaceMesh::Vertex_index > vertices_by_point;

    for (const auto &point : _points_collection->points()) {
        SurfaceMesh::Vertex_index vertex = mesh.add_vertex(point);
        vertices_by_point[point] = vertex;
    }

    for (const auto &face : _points_collection->faces()) {
        SurfaceMesh::Vertex_index vertex0 = vertices_by_point[face->vertex(0)->point()];
        SurfaceMesh::Vertex_index vertex1 = vertices_by_point[face->vertex(1)->point()];
        SurfaceMesh::Vertex_index vertex2 = vertices_by_point[face->vertex(2)->point()];

        mesh.add_face(vertex0, vertex1, vertex2);
    }

    return std::move(mesh);
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeGenerator<PG, NG>::add_point() {
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

GlobeGenerator() -> GlobeGenerator<RandomSpherePointGenerator, AnlNoiseGenerator>;

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_