#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_VIEWER_GEOMETRY_VIEWER_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_VIEWER_GEOMETRY_VIEWER_HPP_

#include "../geometry/helpers.hpp"
#include <QtWidgets/QWidget>
#include <CGAL/Qt/Basic_viewer.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Kernel/global_functions_3.h>
#include <functional>

namespace globe {
    const CGAL::IO::Color BLACK(0, 0, 0);
    const CGAL::IO::Color BLUE(0, 0, 255);
    const CGAL::IO::Color RED(255, 0, 0);

    const double CIRCULAR_ARC_RESOLUTION = 50;

    class GeometryViewer : public CGAL::Basic_viewer {
    public:
        using KeyPressCallback = std::function<void(GeometryViewer &, QKeyEvent *)>;
        struct Config;

        explicit GeometryViewer(Config &&config);

        GeometryViewer(QWidget *parent);

        void set_key_press_callback(KeyPressCallback key_press_callback) {
            _key_press_callback = key_press_callback;
        }

        void add_point(const Point3 &point, const CGAL::IO::Color &color = BLACK);

        void add_segment(const Point3 &source, const Point3 &target, const CGAL::IO::Color &color = BLACK);

        void add_arc(const Point3 &point1, const Point3 &point2, const CGAL::IO::Color &color = BLACK);

        void add_arc(const Arc &arc, const CGAL::IO::Color &color = BLACK);

        void add_text(const Point3 &point, const std::string &text);

        void show();

    protected:
        KeyPressCallback _key_press_callback;

        void keyPressEvent(QKeyEvent *event) override;
    };

    struct GeometryViewer::Config {
        QWidget *parent;
        KeyPressCallback key_press_callback = [](GeometryViewer &, QKeyEvent *) {
        };
    };

    GeometryViewer::GeometryViewer(GeometryViewer::Config &&config) : CGAL::Basic_viewer(
        config.parent, "GeometryViewer", true, true, true, false, false) {
        _key_press_callback = config.key_press_callback;
    }

    GeometryViewer::GeometryViewer(QWidget *parent) : GeometryViewer(Config{.parent = parent}) {
    }

    void GeometryViewer::add_point(const Point3 &point, const CGAL::IO::Color &color) {
        CGAL::Basic_viewer::add_point(point, color);
    }

    void GeometryViewer::add_segment(const Point3 &source, const Point3 &target, const CGAL::IO::Color &color) {
        CGAL::Basic_viewer::add_segment(source, target, color);
    }

    void GeometryViewer::add_arc(const Point3 &point1, const Point3 &point2, const CGAL::Color &color) {
        for (int i = 0; i < CIRCULAR_ARC_RESOLUTION; i++) {
            double t1 = static_cast<double>(i) / CIRCULAR_ARC_RESOLUTION;
            double t2 = static_cast<double>(i + 1) / CIRCULAR_ARC_RESOLUTION;

            Point3 segment_source = spherical_interpolate(point1, point2, t1);
            Point3 segment_target = spherical_interpolate(point1, point2, t2);

            add_segment(segment_source, segment_target, color);
        }
    }

    void GeometryViewer::add_arc(const Arc &arc, const CGAL::Color &color) {
        add_arc(to_point(arc.source()), to_point(arc.target()), color);
    }

    void GeometryViewer::add_text(const Point3 &point, const std::string &text) {
        CGAL::Basic_viewer::add_text(point, text);
    }

    void GeometryViewer::show() {
        CGAL::Basic_viewer::show();
    }

    void GeometryViewer::keyPressEvent(QKeyEvent *event) {
        _key_press_callback(*this, event);
    }
} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_VIEWER_GEOMETRY_VIEWER_HPP_
