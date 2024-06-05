#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_

#include "types.h"
#include "sphere_mesh_generator/sphere_mesh_generator.h"
#include "random_sphere_point_generator/random_sphere_point_generator.h"
#include "points_collection/points_collection.h"
#include <memory>

namespace globe {

class GlobeGenerator {
 public:
    explicit GlobeGenerator(
        double radius = 1.0,
        std::unique_ptr<SphereMeshGenerator> sphere_mesh_generator = nullptr,
        std::unique_ptr<RandomSpherePointGenerator> random_point_generator = nullptr,
        std::unique_ptr<PointsCollection> points_collection = nullptr
    ) :
        _radius(radius),

        _sphere_mesh_generator(
            sphere_mesh_generator ?
                std::move(sphere_mesh_generator) :
                std::make_unique<SphereMeshGenerator>()
        ),

        _random_point_generator(
            random_point_generator ?
                std::move(random_point_generator) :
                std::make_unique<RandomSpherePointGenerator>(radius)
        ),

        _points_collection(
            points_collection
                ?
                std::move(points_collection) :
                std::make_unique<PointsCollection>()
        ) { };

    GlobeGenerator &generate();
    void save_ply(const std::string &filename) const;

 private:
    std::unique_ptr<SphereMeshGenerator> _sphere_mesh_generator;
    std::unique_ptr<RandomSpherePointGenerator> _random_point_generator;
    std::unique_ptr<PointsCollection> _points_collection;

    double _radius;
    std::vector<Point3> _points;
    SurfaceMesh _mesh;

    [[nodiscard]] SurfaceMesh generate_globe_sphere() const;
    void add_random_point();
    void add_point_mesh(Point3 location);
    void add_mesh(SurfaceMesh &mesh);
    [[nodiscard]] bool too_close(const Point3 &point) const;
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
