#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_VIEWER_GEOMETRY_VIEWER_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_VIEWER_GEOMETRY_VIEWER_HPP_

#include "../geometry/helpers.hpp"
#include <QtWidgets/QWidget>
#include <CGAL/Qt/Basic_viewer_qt.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Kernel/global_functions_3.h>

namespace globe {

const CGAL::IO::Color BLACK(0, 0, 0);
const CGAL::IO::Color BLUE(0, 0, 255);
const CGAL::IO::Color RED(255, 0, 0);

class GeometryViewer : public CGAL::Basic_viewer_qt {
 public:
    explicit GeometryViewer(QWidget *parent);

    void add_point(const Point3 &point, const CGAL::IO::Color &color = BLACK);
    void add_segment(const Point3 &source, const Point3 &target, const CGAL::IO::Color &color = BLACK);
    void add_circular_arc(const Point3 &point1, const Point3 &point2, const CGAL::IO::Color &color = BLACK);

    void show();
};

GeometryViewer::GeometryViewer(QWidget *parent) :
    CGAL::Basic_viewer_qt(parent, "GeometryViewer", true, true, true, false, false) {
}

void GeometryViewer::add_point(const Point3 &point, const CGAL::IO::Color &color) {
    CGAL::Basic_viewer_qt::add_point(point, color);
}

void GeometryViewer::add_segment(const Point3 &source, const Point3 &target, const CGAL::IO::Color &color) {
    CGAL::Basic_viewer_qt::add_segment(source, target, color);
}

void GeometryViewer::add_circular_arc(const Point3 &point1, const Point3 &point2, const CGAL::IO::Color &color) {
    const Point3 center(0.0, 0.0, 0.0); // TODO: pass in center

    int num_segments = 50;
    for (int i = 0; i < num_segments; ++i) {
        double t1 = static_cast<double>(i) / num_segments;
        double t2 = static_cast<double>(i + 1) / num_segments;

        Point3 segment_source = spherical_interpolate(point1, point2, t1, center);
        Point3 segment_target = spherical_interpolate(point1, point2, t2, center);

        add_segment(segment_source, segment_target, color);
    }
}

void GeometryViewer::show() {
    CGAL::Basic_viewer_qt::show();
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_VIEWER_GEOMETRY_VIEWER_HPP_