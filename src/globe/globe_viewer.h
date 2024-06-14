#ifndef GLOBEART_SRC_GLOBE_GLOBE_VIEWER_H_
#define GLOBEART_SRC_GLOBE_GLOBE_VIEWER_H_

#include "points_collection/points_collection.h"
#include <QtWidgets/QWidget>
#include <CGAL/Qt/Basic_viewer_qt.h>

namespace globe {

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
    void add_edges();
    void add_voronoi_edges();

    void add_circular_arc(
        const Point3 &point1,
        const Point3 &point2,
        const CGAL::IO::Color &color = CGAL::IO::Color(0, 0, 0)
    );

 private:
    std::unique_ptr<PointsCollection> _points_collection;
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_VIEWER_H_
