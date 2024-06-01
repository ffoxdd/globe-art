#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_

#include "types.h"
#include <memory>

namespace globe {

class GlobeGenerator {
 public:
    explicit GlobeGenerator(double radius = 1.0) : _radius(radius) { };
    GlobeGenerator &generate();
    void save_ply(const std::string &filename) const;

 private:
    double _radius;
    SurfaceMesh _mesh;
    [[nodiscard]] SurfaceMesh generate_globe_sphere() const;
    static SurfaceMesh generate_sphere(double radius, int iterations, Point3 center);
    void add_point(Point3 location);
    void add_mesh(SurfaceMesh &mesh);
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
