#include "globe_viewer.h"
#include "geometry/helpers.h"
#include <CGAL/IO/Color.h>
#include <CGAL/Kernel/global_functions_3.h>

namespace globe {

const CGAL::IO::Color BLUE(0, 0, 255);
const CGAL::IO::Color RED(255, 0, 0);

void GlobeViewer::add_elements() {
//    add_edges();
    add_voronoi_edges();
}

void GlobeViewer::add_edges() {
    for (const auto &segment : _points_collection->segments()) {
        add_circular_arc(segment.source(), segment.target(), BLUE);
    }
}

void GlobeViewer::add_voronoi_edges() {
    for (const auto &arc : _points_collection->dual_arcs()) {
        auto source = to_point(arc.source());
        auto target = to_point(arc.target());

        add_circular_arc(source, target, RED);
    }
}

void GlobeViewer::add_circular_arc(const Point3 &point1, const Point3 &point2, const CGAL::IO::Color &color) {
    const Point3 center(0.0, 0.0, 0.0);

    int num_segments = 50;
    for (int i = 0; i < num_segments; ++i) {
        double t1 = static_cast<double>(i) / num_segments;
        double t2 = static_cast<double>(i + 1) / num_segments;

        Point3 segment_source = spherical_interpolate(point1, point2, t1, center);
        Point3 segment_target = spherical_interpolate(point1, point2, t2, center);

        add_segment(segment_source, segment_target, color);
    }
}

} // namespace globe
