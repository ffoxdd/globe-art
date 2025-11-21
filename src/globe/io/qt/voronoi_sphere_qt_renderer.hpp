#ifndef GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_QT_RENDERER_HPP_
#define GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_QT_RENDERER_HPP_

#include "../../voronoi/spheres/voronoi_sphere.hpp"
#include "qt_viewer.hpp"
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <CGAL/IO/Color.h>
#include <memory>
#include <string>

namespace globe {

class VoronoiSphereQtRenderer {
public:
    explicit VoronoiSphereQtRenderer(QWidget *parent = nullptr, const std::string &window_title = "VoronoiSphere");
    std::unique_ptr<QtViewer> render(const VoronoiSphere &voronoi_sphere);

private:
    void draw_voronoi_sphere(
        const VoronoiSphere &voronoi_sphere,
        QtViewer &viewer
    );

    QWidget *_parent;
    std::string _window_title;
};

inline VoronoiSphereQtRenderer::VoronoiSphereQtRenderer(QWidget *parent, const std::string &window_title) :
    _parent(parent),
    _window_title(window_title) {
}

inline std::unique_ptr<QtViewer> VoronoiSphereQtRenderer::render(const VoronoiSphere &voronoi_sphere) {
    auto viewer = std::make_unique<QtViewer>(_parent, _window_title);
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

