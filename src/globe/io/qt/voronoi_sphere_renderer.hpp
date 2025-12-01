#ifndef GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_RENDERER_HPP_
#define GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_RENDERER_HPP_

#include "viewer.hpp"
#include "../../voronoi/core/voronoi_sphere.hpp"
#include <memory>
#include <string>

namespace globe::io::qt {

class VoronoiSphereRenderer {
public:
    explicit VoronoiSphereRenderer(const std::string &window_title = "VoronoiSphere");
    std::unique_ptr<Viewer> render(const VoronoiSphere &voronoi_sphere);

private:
    void draw_voronoi_sphere(const VoronoiSphere &voronoi_sphere, Viewer &viewer);

    std::string _window_title;
};

inline VoronoiSphereRenderer::VoronoiSphereRenderer(const std::string &window_title) :
    _window_title(window_title) {
}

inline std::unique_ptr<Viewer> VoronoiSphereRenderer::render(const VoronoiSphere &voronoi_sphere) {
    auto viewer = std::make_unique<Viewer>(nullptr, _window_title);
    draw_voronoi_sphere(voronoi_sphere, *viewer);
    viewer->show();
    return viewer;
}

inline void VoronoiSphereRenderer::draw_voronoi_sphere(
    const VoronoiSphere &voronoi_sphere,
    Viewer &viewer
) {
    viewer.clear();

    for (const auto &cell : voronoi_sphere.cells()) {
        for (const auto &arc : cell.arcs()) {
            viewer.add_arc(arc, Color(140, 140, 140));
        }
    }
}

} // namespace globe::io::qt

#endif //GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_RENDERER_HPP_

