#ifndef GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_DRAWER_HPP_
#define GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_DRAWER_HPP_

#include "viewer.hpp"
#include "../../voronoi/spherical/core/sphere.hpp"
#include <memory>
#include <string>
#include <chrono>

namespace globe::io::qt {

using voronoi::spherical::Sphere;

class SphereDrawer {
public:
    SphereDrawer(const std::string &window_title = "Sphere");
    void show();
    void show(const Sphere &sphere);
    void update(const Sphere &sphere);

private:
    std::string _window_title;
    std::unique_ptr<Viewer> _viewer;
    bool _has_centered = false;

    void ensure_viewer();
    void draw(const Sphere &sphere);
    void center_if_needed();
};

inline SphereDrawer::SphereDrawer(const std::string &window_title) :
    _window_title(window_title) {
}

inline void SphereDrawer::ensure_viewer() {
    if (!_viewer) {
        _viewer = std::make_unique<Viewer>(nullptr, _window_title);
    }
}

inline void SphereDrawer::show() {
    ensure_viewer();
    _viewer->show();
}

inline void SphereDrawer::show(const Sphere &sphere) {
    ensure_viewer();
    _viewer->clear();
    draw(sphere);
    _viewer->show();
}

inline void SphereDrawer::draw(const Sphere &sphere) {
    Color face_color(200, 200, 200);
    Color edge_color(60, 60, 60);

    for (const auto &cell : sphere.cells()) {
        VectorS2 centroid = cell.centroid();
        std::vector<VectorS2> points(cell.points().begin(), cell.points().end());

        for (size_t i = 0; i < points.size(); ++i) {
            size_t next = (i + 1) % points.size();
            _viewer->add_triangle(centroid, points[i], points[next], face_color);
        }
    }

    for (const auto &arc : sphere.arcs()) {
        _viewer->add_arc(arc, edge_color);
    }
}

inline void SphereDrawer::center_if_needed() {
    if (!_has_centered) {
        _viewer->center_on_unit_sphere();
        _has_centered = true;
    }
}

inline void SphereDrawer::update(const Sphere &sphere) {
    ensure_viewer();
    _viewer->clear();
    draw(sphere);
    center_if_needed();
    _viewer->redraw();
}

} // namespace globe::io::qt

#endif //GLOBEART_SRC_GLOBE_IO_QT_VORONOI_SPHERE_DRAWER_HPP_
