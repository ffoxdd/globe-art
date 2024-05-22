#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_

#include "types.h"

namespace globe {

class GlobeGenerator {
 public:
    void generate();
    void save_ply(const std::string &filename) const;

 private:
    SurfaceMesh _mesh;
    static const SurfaceMesh &generate_globe_sphere();
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
