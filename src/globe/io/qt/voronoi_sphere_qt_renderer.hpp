#ifndef GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_QT_RENDERER_HPP_
#define GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_QT_RENDERER_HPP_

#include "../../voronoi/core/voronoi_sphere.hpp"
#include "qt_viewer.hpp"
#include <CGAL/IO/Color.h>
#include <memory>
#include <string>

namespace globe {

class VoronoiSphereQtRenderer {
public:
    explicit VoronoiSphereQtRenderer(const std::string &window_title = "VoronoiSphere");
    std::unique_ptr<QtViewer> render(const VoronoiSphere &voronoi_sphere);

private:
    void draw_voronoi_sphere(
        const VoronoiSphere &voronoi_sphere,
        QtViewer &viewer
    );

    std::string _window_title;
};

inline VoronoiSphereQtRenderer::VoronoiSphereQtRenderer(const std::string &window_title) :
    _window_title(window_title) {
}

inline std::unique_ptr<QtViewer> VoronoiSphereQtRenderer::render(const VoronoiSphere &voronoi_sphere) {
    auto viewer = std::make_unique<QtViewer>(nullptr, _window_title);
    draw_voronoi_sphere(voronoi_sphere, *viewer);
    viewer->show();
    return viewer;
}

inline void VoronoiSphereQtRenderer::draw_voronoi_sphere(
    const VoronoiSphere &voronoi_sphere,
    QtViewer &viewer
) {
    viewer.clear();

    for (const auto &arc : voronoi_sphere.dual_arcs()) {
        viewer.add_arc(arc, CGAL::IO::Color(140, 140, 140));
    }
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_QT_RENDERER_HPP_

