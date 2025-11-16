#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_

#include "../types.hpp"
#include "../point_generator/point_generator.hpp"
#include "../point_generator/random_sphere_point_generator.hpp"
#include "../points_collection/points_collection.hpp"
#include "../noise_generator/scalar_field.hpp"
#include "../noise_generator/noise_field.hpp"
#include "spherical_polygon.hpp"
#include "sample_point_generator/bounding_box_sample_point_generator.hpp"
#include "centroid_calculator.hpp"
#include "mass_calculator.hpp"
#include "../noise_generator/interval.hpp"
#include <queue>
#include <vector>
#include <map>
#include <fstream>
#include <stdexcept>
#include <format>
#include <string>
#include <utility>
#include <iostream>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

namespace globe {

const Interval DENSITY_FIELD_INTERVAL = Interval(1, 100);
const int POINT_COUNT = 250;

template<
    PointGenerator PG = RandomSpherePointGenerator,
    ScalarField DF = NoiseField
>
class GlobeGenerator {
 public:
    GlobeGenerator(
        PG point_generator = PG(RandomSpherePointGenerator(1.0)),
        PointsCollection points_collection = PointsCollection(),
        DF density_field = DF(NoiseField())
    );

    void build();
    void initialize();
    void add_points();
    void relax(int count = 1);

    void save_ply(const std::string &filename) const;

    auto dual_arcs();
    auto dual_neighborhoods();

    struct CellDebugInfo {
        Point3 site;
        std::vector<Arc> dual_cell_arcs;
        double mass;
        Point3 centroid;
        double theta_low;
        double theta_high;
        double z_low;
        double z_high;
    };

    std::vector<CellDebugInfo> cell_debug_info();

 private:
    PG _point_generator;
    PointsCollection _points_collection;
    DF _density_field;

    void normalize_density_field();
    void calculate_target_mass();
    void add_point();
    std::vector<Point3> sample_points(size_t n);
    [[nodiscard]] SurfaceMesh triangulation_mesh() const;
    static void save_mesh_ply(SurfaceMesh &mesh, const std::string &filename);
    Point3 centroid(const SphericalPolygon &spherical_polygon);
    double mass(const SphericalPolygon &spherical_polygon);
    double total_mass();
    double average_mass();
    Point3 optimize_vertex_position(size_t index, double target_mass);
    void adjust_mass();
    void adjust_centroids();
};

template<PointGenerator PG, ScalarField DF>
GlobeGenerator<PG, DF>::GlobeGenerator(
    PG point_generator,
    PointsCollection points_collection,
    DF density_field
) :
    _point_generator(std::move(point_generator)),
    _points_collection(std::move(points_collection)),
    _density_field(std::move(density_field)) {
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::build() {
    initialize();
    add_points();
    relax();
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::save_ply(const std::string &filename) const {
    SurfaceMesh mesh = triangulation_mesh();
    save_mesh_ply(mesh, filename);
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::initialize() {
    normalize_density_field();
    calculate_target_mass();
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::normalize_density_field() {
    _density_field.normalize(sample_points(1000), DENSITY_FIELD_INTERVAL);
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::calculate_target_mass() {
    SphericalCircle3 circle(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(1, 0, 0));

    SphericalPolygon spherical_polygon = SphericalPolygon(
        std::vector<Arc>{
            Arc(circle, SphericalPoint3(1, 0, 0), SphericalPoint3(0, 1, 0)),
            Arc(circle, SphericalPoint3(0, 1, 0), SphericalPoint3(0, 0, 1)),
            Arc(circle, SphericalPoint3(0, 0, 1), SphericalPoint3(1, 0, 0))
        }
    );


}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::add_points() {
    for (int i = 0; i < POINT_COUNT; i++) {
        add_point();
    }
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::relax(int count) {
    for (int i = 0; i < count; i++) {
        adjust_mass();
//        adjust_centroids();
    }
}

struct VoronoiCell {
    size_t index;
    double mass{};
};

struct MinMassComparator {
    bool operator()(const VoronoiCell &a, const VoronoiCell &b) const {
        return a.mass > b.mass;
    }
};

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::adjust_mass() {
    double target_mass = average_mass();

    std::priority_queue<VoronoiCell, std::vector<VoronoiCell>, MinMassComparator> heap;
    for (size_t i = 0; i < _points_collection.size(); i++) {
        double cell_mass = mass(SphericalPolygon(_points_collection.dual_cell_arcs(i)));
        heap.push({i, cell_mass});
    }

    while (!heap.empty()) {
        size_t i = heap.top().index;
        heap.pop();

        Point3 optimized_position = optimize_vertex_position(i, target_mass);
        _points_collection.update_site(i, optimized_position);
    }
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::adjust_centroids() {
    using DualNeighborhoodIterator = decltype(_points_collection.dual_neighborhoods().begin());
    using DualNeighborhoodType = typename std::iterator_traits<DualNeighborhoodIterator>::value_type;

    std::vector<DualNeighborhoodType> temp_dual_neighborhoods(
        _points_collection.dual_neighborhoods().begin(), _points_collection.dual_neighborhoods().end()
    );

    std::vector<Point3> new_points(temp_dual_neighborhoods.size());

    oneapi::tbb::parallel_for(oneapi::tbb::blocked_range<size_t>(0, temp_dual_neighborhoods.size()),
        [&](const oneapi::tbb::blocked_range<size_t> &range) {
            for (size_t i = range.begin(); i < range.end(); ++i) {
                const SphericalPolygon spherical_polygon(temp_dual_neighborhoods[i].dual_cell_arcs);
                new_points[i] = this->centroid(spherical_polygon);
            }
        }
    );

    _points_collection.reset(new_points);
}

template<PointGenerator PG, ScalarField DF>
Point3 GlobeGenerator<PG, DF>::centroid(const SphericalPolygon &spherical_polygon) {
    auto bounding_box = spherical_polygon.bounding_box();
    auto generator = BoundingBoxSamplePointGenerator(bounding_box);

    return CentroidCalculator<DF, BoundingBoxSamplePointGenerator>(
        spherical_polygon,
        _density_field,
        std::move(generator)
    ).centroid();
}

template<PointGenerator PG, ScalarField DF>
double GlobeGenerator<PG, DF>::mass(const SphericalPolygon &spherical_polygon) {
    auto bounding_box = spherical_polygon.bounding_box();
    auto generator = BoundingBoxSamplePointGenerator(bounding_box);

    return MassCalculator<DF, BoundingBoxSamplePointGenerator>(
        std::ref(spherical_polygon),
        _density_field,
        std::move(generator)
    ).mass();
}

template<PointGenerator PG, ScalarField DF>
double GlobeGenerator<PG, DF>::total_mass() {
    auto bounding_box = SphericalBoundingBox(Interval(0, 2 * M_PI), Interval(-1, 1));
    auto generator = BoundingBoxSamplePointGenerator(bounding_box);

    return MassCalculator<DF, BoundingBoxSamplePointGenerator>(
        std::nullopt,
        _density_field,
        std::move(generator)
    ).mass();
}

template<PointGenerator PG, ScalarField DF>
double GlobeGenerator<PG, DF>::average_mass() {
    return total_mass() / _points_collection.size();
}

template<PointGenerator PG, ScalarField DF>
Point3 GlobeGenerator<PG, DF>::optimize_vertex_position(size_t index, double target_mass) {
    auto objective = [&](const Point3& candidate) {
        _points_collection.update_site(index, candidate);
        double cell_mass = mass(SphericalPolygon(_points_collection.dual_cell_arcs(index)));
        return std::pow(cell_mass - target_mass, 2);
    };

    // TODO: Implement proper Nelder-Mead
    // For now, just evaluate current position and return it
    Point3 current_position = _points_collection.site(index);
    double current_error = objective(current_position);

    return current_position;
}

template<PointGenerator PG, ScalarField DF>
std::vector<typename GlobeGenerator<PG, DF>::CellDebugInfo> GlobeGenerator<PG, DF>::cell_debug_info() {
    std::vector<CellDebugInfo> debug_info;

    for (const auto &vertex : _points_collection.vertices()) {
        auto dual_cell = _points_collection.dual_cell_arcs(vertex);

        if (dual_cell.empty()) {
            continue;
        }

        SphericalPolygon spherical_polygon(dual_cell);
        double cell_mass = mass(spherical_polygon);
        Point3 centroid_value = centroid(spherical_polygon);
        auto bounding_box = spherical_polygon.bounding_box();

        debug_info.push_back(CellDebugInfo{
            .site = vertex->point(),
            .dual_cell_arcs = std::move(dual_cell),
            .mass = cell_mass,
            .centroid = centroid_value,
            .theta_low = bounding_box.theta_interval().low(),
            .theta_high = bounding_box.theta_interval().high(),
            .z_low = bounding_box.z_interval().low(),
            .z_high = bounding_box.z_interval().high()
        });
    }

    return debug_info;
}

template<PointGenerator PG, ScalarField DF>
std::vector<Point3> GlobeGenerator<PG, DF>::sample_points(size_t n) {
    std::vector<Point3> points;

    for (size_t i = 0; i < n; i++) {
        points.push_back(_point_generator.generate());
    }

    return points;
}

template<PointGenerator PG, ScalarField DF>
auto GlobeGenerator<PG, DF>::dual_arcs() {
    return _points_collection.dual_arcs();
}

template<PointGenerator PG, ScalarField DF>
auto GlobeGenerator<PG, DF>::dual_neighborhoods() {
    return _points_collection.dual_neighborhoods();
}

template<PointGenerator PG, ScalarField DF>
SurfaceMesh GlobeGenerator<PG, DF>::triangulation_mesh() const {
    SurfaceMesh mesh;
    std::map<Point3, SurfaceMesh::Vertex_index> vertices_by_point;

    for (const auto &point : _points_collection.points()) {
        SurfaceMesh::Vertex_index vertex = mesh.add_vertex(point);
        vertices_by_point[point] = vertex;
    }

    for (const auto &face : _points_collection.faces()) {
        SurfaceMesh::Vertex_index vertex0 = vertices_by_point[face->vertex(0)->point()];
        SurfaceMesh::Vertex_index vertex1 = vertices_by_point[face->vertex(1)->point()];
        SurfaceMesh::Vertex_index vertex2 = vertices_by_point[face->vertex(2)->point()];

        mesh.add_face(vertex0, vertex1, vertex2);
    }

    return std::move(mesh);
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::add_point() {
    Point3 point = _point_generator.generate();
    _points_collection.insert(point);
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::save_mesh_ply(
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

GlobeGenerator() -> GlobeGenerator<RandomSpherePointGenerator, NoiseField>;

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_