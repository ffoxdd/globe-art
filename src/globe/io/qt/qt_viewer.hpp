#ifndef GLOBEART_SRC_GLOBE_IO_QT_QT_VIEWER_HPP_
#define GLOBEART_SRC_GLOBE_IO_QT_QT_VIEWER_HPP_

#include <CGAL/Qt/Basic_viewer.h>
#include <CGAL/Graphics_scene.h>
#include <CGAL/IO/Color.h>
#include <QtWidgets/QWidget>
#include <QtGui/QKeyEvent>
#include <functional>
#include "../../cgal_types.hpp"
#include "../../geometry/spherical/spherical_arc.hpp"
#include "../cgal_helpers.hpp"

namespace globe {

const CGAL::IO::Color BLACK(0, 0, 0);
const CGAL::IO::Color BLUE(0, 0, 255);
const CGAL::IO::Color RED(255, 0, 0);

constexpr double CIRCULAR_ARC_RESOLUTION = 50;

class QtViewer final : public ::CGAL::Qt::Basic_viewer {
public:
    using KeyPressCallback = std::function<void(QtViewer &, QKeyEvent *)>;

    explicit QtViewer(
        QWidget *parent,
        const std::string &window_title = "QtViewer",
        KeyPressCallback key_press_callback = [](QtViewer &, QKeyEvent *) {}
    );

    inline void set_key_press_callback(const KeyPressCallback &key_press_callback) {
        _key_press_callback = key_press_callback;
    }

    void add_point(const Point3 &point, const CGAL::IO::Color &color = BLACK);
    void add_segment(const Point3 &source, const Point3 &target, const CGAL::IO::Color &color = BLACK);
    void add_arc(const Point3 &point1, const Point3 &point2, const CGAL::IO::Color &color = BLACK);
    void add_arc(const SphericalArc &arc, const CGAL::IO::Color &color = BLACK);
    void add_text(const Point3 &point, const std::string &text);
    void clear();
    void show();
    void redraw() override;

protected:
    KeyPressCallback _key_press_callback;
    void keyPressEvent(QKeyEvent *event) override;
    CGAL::Graphics_scene _scene;
};

inline QtViewer::QtViewer(
    QWidget *parent,
    const std::string &window_title,
    KeyPressCallback key_press_callback
)
    : ::CGAL::Qt::Basic_viewer(parent, _scene, window_title.c_str()),
    _key_press_callback(key_press_callback) {
    draw_vertices(true);
    size_vertices(5.0f);
}

inline void QtViewer::add_point(const Point3 &point, const CGAL::IO::Color &color) {
    _scene.add_point(point, color);
}

inline void QtViewer::add_segment(const Point3 &source, const Point3 &target, const CGAL::IO::Color &color) {
    _scene.add_segment(source, target, color);
}

inline void QtViewer::add_arc(const Point3 &point1, const Point3 &point2, const CGAL::IO::Color &color) {
    for (int i = 0; i < CIRCULAR_ARC_RESOLUTION; i++) {
        const double t1 = static_cast<double>(i) / CIRCULAR_ARC_RESOLUTION;
        const double t2 = static_cast<double>(i + 1) / CIRCULAR_ARC_RESOLUTION;

        Point3 segment_source = io::interpolate(point1, point2, t1);
        Point3 segment_target = io::interpolate(point1, point2, t2);

        add_segment(segment_source, segment_target, color);
    }
}

inline void QtViewer::add_arc(const SphericalArc &arc, const CGAL::IO::Color &color) {
    add_arc(to_cgal_point(arc.source()), to_cgal_point(arc.target()), color);
}

inline void QtViewer::add_text(const Point3 &point, const std::string &text) {
    _scene.add_text(point, text);
}

inline void QtViewer::clear() {
    _scene.clear();
}

inline void QtViewer::show() {
    ::CGAL::Qt::Basic_viewer::show();
}

inline void QtViewer::redraw() {
    ::CGAL::Qt::Basic_viewer::redraw();
}

inline void QtViewer::keyPressEvent(QKeyEvent *event) {
    _key_press_callback(*this, event);
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_IO_QT_QT_VIEWER_HPP_

