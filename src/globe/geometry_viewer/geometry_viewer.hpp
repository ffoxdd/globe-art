#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_VIEWER_GEOMETRY_VIEWER_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_VIEWER_GEOMETRY_VIEWER_HPP_

#include <CGAL/Qt/Basic_viewer.h>
#include <CGAL/Graphics_scene.h>
#include <CGAL/IO/Color.h>
#include <QtWidgets/QWidget>
#include <QtGui/QKeyEvent>
#include <functional>
#include "../geometry/helpers.hpp"

namespace globe {
    const CGAL::IO::Color BLACK(0, 0, 0);
    const CGAL::IO::Color BLUE(0, 0, 255);
    const CGAL::IO::Color RED(255, 0, 0);

    constexpr double CIRCULAR_ARC_RESOLUTION = 50;

    class GeometryViewer final : public ::CGAL::Qt::Basic_viewer {
    public:
        using KeyPressCallback = std::function<void(GeometryViewer &, QKeyEvent *)>;
        struct Config;

        explicit GeometryViewer(Config &&config);
        explicit GeometryViewer(QWidget *parent);

        void set_key_press_callback(const KeyPressCallback &key_press_callback) {
            _key_press_callback = key_press_callback;
        }

        void add_point(const Point3 &point, const CGAL::IO::Color &color = BLACK);
        void add_segment(const Point3 &source, const Point3 &target, const CGAL::IO::Color &color = BLACK);
        void add_arc(const Point3 &point1, const Point3 &point2, const CGAL::IO::Color &color = BLACK);
        void add_arc(const Arc &arc, const CGAL::IO::Color &color = BLACK);
        void add_text(const Point3 &point, const std::string &text);
        void clear();
        void show();
        void redraw() override;

    protected:
        KeyPressCallback _key_press_callback;
        void keyPressEvent(QKeyEvent *event) override;
        CGAL::Graphics_scene _scene;
    };

    struct GeometryViewer::Config {
        QWidget *parent;
        KeyPressCallback key_press_callback = [](GeometryViewer &, QKeyEvent *) {
        };
    };

    inline GeometryViewer::GeometryViewer(Config &&config)
        : ::CGAL::Qt::Basic_viewer(config.parent, _scene, "GeometryViewer"),
        _key_press_callback(config.key_press_callback) {
    }

    inline GeometryViewer::GeometryViewer(QWidget *parent) : GeometryViewer(Config{.parent = parent}) {
    }

    inline void GeometryViewer::add_point(const Point3 &point, const CGAL::IO::Color &color) {
        _scene.add_point(point, color);
    }

    inline void GeometryViewer::add_segment(const Point3 &source, const Point3 &target, const CGAL::IO::Color &color) {
        _scene.add_segment(source, target, color);
    }

    inline void GeometryViewer::add_arc(const Point3 &point1, const Point3 &point2, const CGAL::Color &color) {
        for (int i = 0; i < CIRCULAR_ARC_RESOLUTION; i++) {
            const double t1 = static_cast<double>(i) / CIRCULAR_ARC_RESOLUTION;
            const double t2 = static_cast<double>(i + 1) / CIRCULAR_ARC_RESOLUTION;

            Point3 segment_source = spherical_interpolate(point1, point2, t1);
            Point3 segment_target = spherical_interpolate(point1, point2, t2);

            add_segment(segment_source, segment_target, color);
        }
    }

    inline void GeometryViewer::add_arc(const Arc &arc, const CGAL::Color &color) {
        add_arc(to_point(arc.source()), to_point(arc.target()), color);
    }

    inline void GeometryViewer::add_text(const Point3 &point, const std::string &text) {
        _scene.add_text(point, text);
    }

    inline void GeometryViewer::clear() {
        _scene.clear();
    }

    inline void GeometryViewer::show() {
        ::CGAL::Qt::Basic_viewer::show();
    }

    inline void GeometryViewer::redraw() {
        ::CGAL::Qt::Basic_viewer::redraw();
    }

    inline void GeometryViewer::keyPressEvent(QKeyEvent *event) {
        _key_press_callback(*this, event);
    }
} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_VIEWER_GEOMETRY_VIEWER_HPP_
