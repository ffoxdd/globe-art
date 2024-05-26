#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_

#include "types.h"
#include <memory>

namespace globe {

class GlobeGenerator {
 public:
    void generate();
    void save_ply(const std::string &filename) const;

 private:
    std::unique_ptr<SurfaceMesh> _mesh;
    static std::unique_ptr<SurfaceMesh> generate_globe_sphere();
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_H_
