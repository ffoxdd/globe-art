#ifndef GLOBEART_SRC_GLOBE_IO_TEXT_SPHERE_REPOSITORY_HPP_
#define GLOBEART_SRC_GLOBE_IO_TEXT_SPHERE_REPOSITORY_HPP_

#include "../../voronoi/spherical/core/sphere.hpp"
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace globe::io::text {

using voronoi::spherical::Sphere;

class SphereRepository {
 public:
    static void save(const Sphere& sphere, const std::string& path) {
        std::ofstream file(path);
        if (!file) {
            throw std::runtime_error("Failed to open file for writing: " + path);
        }

        file << std::setprecision(17);

        for (size_t i = 0; i < sphere.size(); ++i) {
            cgal::Point3 site = sphere.site(i);
            file << site.x() << " " << site.y() << " " << site.z() << "\n";
        }
    }

    static std::unique_ptr<Sphere> load(const std::string& path) {
        std::ifstream file(path);
        if (!file) {
            throw std::runtime_error("Failed to open file for reading: " + path);
        }

        auto sphere = std::make_unique<Sphere>();
        std::string line;

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }

            std::istringstream iss(line);
            double x, y, z;

            if (!(iss >> x >> y >> z)) {
                throw std::runtime_error("Failed to parse line: " + line);
            }

            sphere->insert(cgal::Point3(x, y, z));
        }

        return sphere;
    }
};

} // namespace globe::io::text

#endif //GLOBEART_SRC_GLOBE_IO_TEXT_SPHERE_REPOSITORY_HPP_
