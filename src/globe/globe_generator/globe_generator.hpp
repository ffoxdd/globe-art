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
#include "area_calculator.hpp"
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
        double capacity;
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
    void calculate_target_capacity();
    void add_point();
    std::vector<Point3> sample_points(size_t n);
    [[nodiscard]] SurfaceMesh triangulation_mesh() const;
    static void save_mesh_ply(SurfaceMesh &mesh, const std::string &filename);
    Point3 centroid(const SphericalPolygon &spherical_polygon);
    double area(const SphericalPolygon &spherical_polygon);
    void adjust_capacity();
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
    calculate_target_capacity();
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::normalize_density_field() {
    _density_field.normalize(sample_points(1000), DENSITY_FIELD_INTERVAL);
}

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::calculate_target_capacity() {
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
        adjust_capacity();
//        adjust_centroids();
    }
}

struct VoronoiCell {
    VertexHandle vertex;
    double capacity{};
};

struct MinCapacityComparator {
    bool operator()(const VoronoiCell &a, const VoronoiCell &b) const {
        return a.capacity > b.capacity;
    }
};

template<PointGenerator PG, ScalarField DF>
void GlobeGenerator<PG, DF>::adjust_capacity() {
    std::priority_queue<VoronoiCell, std::vector<VoronoiCell>, MinCapacityComparator> min_capacity_heap;

    for (const auto &vertex : _points_collection.vertices()) {
        const SphericalPolygon spherical_polygon(_points_collection.dual_cell_arcs(vertex));

        double capacity = area(spherical_polygon);
        // std::cout << "capacity: " << capacity << std::endl;

        VoronoiCell voronoi_cell{vertex, capacity};

        min_capacity_heap.push(voronoi_cell);
    }

    const VoronoiCell &min_cell = min_capacity_heap.top();

    auto min_cell_arcs = _points_collection.dual_cell_arcs(min_cell.vertex);
    if (!min_cell_arcs.empty()) {
        SphericalPolygon polygon(min_cell_arcs);
        auto polygon_centroid = centroid(polygon);
        auto bbox = polygon.bounding_box();

        const Point3 &site = min_cell.vertex->point();
        std::cout << "---- Min Capacity Cell Debug ----" << std::endl;
        std::cout << "Capacity      : " << min_cell.capacity << std::endl;
        std::cout << "Site          : (" << site.x() << ", " << site.y() << ", " << site.z() << ")" << std::endl;
        std::cout << "Centroid      : (" << polygon_centroid.x() << ", " << polygon_centroid.y() << ", "
                  << polygon_centroid.z() << ")" << std::endl;
        std::cout << "Arc Count     : " << min_cell_arcs.size() << std::endl;
        std::cout << "Theta Interval: [" << bbox.theta_interval().low() << ", " << bbox.theta_interval().high() << "]"
                  << std::endl;
        std::cout << "Z Interval    : [" << bbox.z_interval().low() << ", " << bbox.z_interval().high() << "]"
                  << std::endl;

        for (std::size_t i = 0; i < min_cell_arcs.size(); ++i) {
            auto source = to_point(min_cell_arcs[i].source());
            auto target = to_point(min_cell_arcs[i].target());
            std::cout << "  Arc " << i << ": source(" << source.x() << ", " << source.y() << ", " << source.z()
                      << ") -> target(" << target.x() << ", " << target.y() << ", " << target.z() << ")"
                      << std::endl;
        }
        std::cout << "---------------------------------" << std::endl;
    }

    // ok, now we need to move the cell's vertex, minimizing the capacity error
    // for now we'll calculate the error globally
    // an optimization is to search only the faces that have changed after each vertex movement

    // steps

    // calculate the global mass
    // determine the optimal capacity per cell
    // initialize a min-heap with all vertices keyed on their capacity difference
    // take the vertex whose dual cell has the minimal capacity difference
    // minimize the capacity error w/ the downhill simplex method
    // move the vertex around, within the area of its dual face neighborhood
    // recalculate the capacities of adjacent cells
    // you can just do a few minimization steps as opposed to finding the true minimum
    // repeat until no vertices move

    // need:
    // min-heap
    // * vertex accessor
    // downhill simplex algorithm
    // * ability to move a vertex
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
double GlobeGenerator<PG, DF>::area(const SphericalPolygon &spherical_polygon) {
    auto bounding_box = spherical_polygon.bounding_box();
    auto generator = BoundingBoxSamplePointGenerator(bounding_box);

    return AreaCalculator<DF, BoundingBoxSamplePointGenerator>(
        spherical_polygon,
        _density_field,
        std::move(generator)
    ).area();
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
        double capacity = area(spherical_polygon);
        Point3 centroid_value = centroid(spherical_polygon);
        auto bounding_box = spherical_polygon.bounding_box();

        debug_info.push_back(CellDebugInfo{
            .site = vertex->point(),
            .dual_cell_arcs = std::move(dual_cell),
            .capacity = capacity,
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