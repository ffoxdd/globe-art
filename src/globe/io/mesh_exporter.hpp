#ifndef GLOBEART_SRC_GLOBE_IO_MESH_EXPORTER_HPP_
#define GLOBEART_SRC_GLOBE_IO_MESH_EXPORTER_HPP_

#include <CGAL/IO/write_ply_points.h>
#include <fstream>
#include <stdexcept>
#include <format>
#include <string>

namespace globe {

class MeshExporter {
 public:
    template<typename Mesh>
    static void save_ply(const Mesh &mesh, const std::string &filename);
};

template<typename Mesh>
inline void MeshExporter::save_ply(const Mesh &mesh, const std::string &filename) {
    std::ofstream stream(filename);

    if (!stream) {
        throw std::runtime_error(std::format("Cannot open file for writing: {}", filename));
    }

    bool success = CGAL::IO::write_PLY(stream, mesh);

    if (!success) {
        throw std::runtime_error(std::format("Cannot write file: {}", filename));
    }
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_IO_MESH_EXPORTER_HPP_

