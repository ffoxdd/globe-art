#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_

#include "types.h"
#include "sphere_mesh_generator/sphere_mesh_generator.h"
#include <memory>
#include <CGAL/point_generators_3.h>

namespace globe {

typedef CGAL::Random_points_on_sphere_3<Point3> RandomSpherePointGenerator;

class GlobeGenerator {
 public:
    explicit GlobeGenerator(
        double radius = 1.0,
        std::unique_ptr<SphereMeshGenerator> sphere_mesh_generator = std::make_unique<SphereMeshGenerator>()
    ) :
        _radius(radius),
        _sphere_mesh_generator(std::move(sphere_mesh_generator)),
        _point_generator(RandomSpherePointGenerator(radius)) { };

    GlobeGenerator &generate();
    void save_ply(const std::string &filename) const;

 private:
    std::unique_ptr<SphereMeshGenerator> _sphere_mesh_generator;
    double _radius;
    SurfaceMesh _mesh;
    RandomSpherePointGenerator _point_generator;

    [[nodiscard]] SurfaceMesh generate_globe_sphere() const;
    void add_random_point();
    void add_point(Point3 location);
    Point3 random_point();
    void add_mesh(SurfaceMesh &mesh);

};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
