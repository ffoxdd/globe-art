#ifndef GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_DRAWER_HPP_
#define GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_DRAWER_HPP_

#include "viewer.hpp"
#include "../../voronoi/core/voronoi_sphere.hpp"
#include <memory>
#include <string>

namespace globe::io::qt {

class VoronoiSphereDrawer {
public:
    VoronoiSphereDrawer(const std::string &window_title = "VoronoiSphere");
    void show(const VoronoiSphere &voronoi_sphere);

private:
    std::string _window_title;
    std::unique_ptr<Viewer> _viewer;

    void draw(const VoronoiSphere &voronoi_sphere);
};

inline VoronoiSphereDrawer::VoronoiSphereDrawer(const std::string &window_title) :
    _window_title(window_title) {
}

inline void VoronoiSphereDrawer::show(const VoronoiSphere &voronoi_sphere) {
    _viewer = std::make_unique<Viewer>(nullptr, _window_title);
    draw(voronoi_sphere);
    _viewer->show();
}

inline void VoronoiSphereDrawer::draw(const VoronoiSphere &voronoi_sphere) {
    for (const auto &arc : voronoi_sphere.arcs()) {
        _viewer->add_arc(arc, Color(140, 140, 140));
    }
}

} // namespace globe::io::qt

#endif //GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_DRAWER_HPP_

