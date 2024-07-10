#ifndef GLOBEART_SRC_GLOBE_GLOBE_VIEWER_H_
#define GLOBEART_SRC_GLOBE_GLOBE_VIEWER_H_

#include "points_collection/points_collection.hpp"
#include "geometry/helpers.hpp"
#include <QtWidgets/QWidget>
#include <CGAL/Qt/Basic_viewer_qt.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Kernel/global_functions_3.h>

namespace globe {

const CGAL::IO::Color BLACK(0, 0, 0);
const CGAL::IO::Color BLUE(0, 0, 255);
const CGAL::IO::Color RED(255, 0, 0);

class GlobeViewer : public CGAL::Basic_viewer_qt {
 public:
    explicit GlobeViewer(
        QWidget *parent,
        std::unique_ptr<PointsCollection> points_collection = nullptr // TODO: inject the globe generator instead
    ) :
        CGAL::Basic_viewer_qt(parent, "Spherical Triangulation GlobeViewer", true, true, true, false, false),

        _points_collection(
            points_collection ?
                std::move(points_collection) :
                std::make_unique<PointsCollection>()
        ) {
        add_elements();
    }

 protected:
    void add_elements();
    void add_voronoi_edges();

    void add_circular_arc(
        const Point3 &point1,
        const Point3 &point2,
        const CGAL::IO::Color &color = CGAL::IO::Color(0, 0, 0)
    );

 private:
    std::unique_ptr<PointsCollection> _points_collection;
};

void GlobeViewer::add_elements() {
    add_voronoi_edges();
}

void GlobeViewer::add_voronoi_edges() {
    for (const auto &arc : _points_collection->dual_arcs()) {
        auto source = to_point(arc.source());
        auto target = to_point(arc.target());

        add_circular_arc(source, target, BLACK);
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

#endif //GLOBEART_SRC_GLOBE_GLOBE_VIEWER_H_
