#ifndef GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_DRAWER_HPP_
#define GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_DRAWER_HPP_

#include "viewer.hpp"
#include "../../voronoi/spherical/core/sphere.hpp"
#include <memory>
#include <string>

namespace globe::io::qt {

using voronoi::spherical::Sphere;

class SphereDrawer {
public:
    SphereDrawer(const std::string &window_title = "Sphere");
    void show(const Sphere &sphere);

private:
    std::string _window_title;
    std::unique_ptr<Viewer> _viewer;

    void draw(const Sphere &sphere);
};

inline SphereDrawer::SphereDrawer(const std::string &window_title) :
    _window_title(window_title) {
}

inline void SphereDrawer::show(const Sphere &sphere) {
    _viewer = std::make_unique<Viewer>(nullptr, _window_title);
    draw(sphere);
    _viewer->show();
}

inline void SphereDrawer::draw(const Sphere &sphere) {
    for (const auto &arc : sphere.arcs()) {
        _viewer->add_arc(arc, Color(140, 140, 140));
    }
}

} // namespace globe::io::qt

#endif //GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_DRAWER_HPP_
