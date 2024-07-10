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

template<
    PointGenerator PG = RandomSpherePointGenerator,
    NoiseGenerator NG = AnlNoiseGenerator<PG>
>
class GlobeViewer : public CGAL::Basic_viewer_qt {
 public:
    struct Config;

    GlobeViewer();
    explicit GlobeViewer(Config &&config);

    void add_elements();

 protected:
    void add_voronoi_edges();
    void add_circular_arc(const Point3 &point1, const Point3 &point2, const CGAL::IO::Color &color = BLACK);

 private:
    std::unique_ptr<GlobeGenerator<PG, NG>> _globe_generator{};
};

template<PointGenerator PG, NoiseGenerator NG>
struct GlobeViewer<PG, NG>::Config {
    QWidget *parent = nullptr;
    std::unique_ptr<GlobeGenerator<PG, NG>> globe_generator = std::make_unique<GlobeGenerator<PG, NG>>();
};

template<PointGenerator PG, NoiseGenerator NG>
GlobeViewer<PG, NG>::GlobeViewer() : GlobeViewer(Config()) {
}

template<PointGenerator PG, NoiseGenerator NG>
GlobeViewer<PG, NG>::GlobeViewer(GlobeViewer::Config &&config) :
    CGAL::Basic_viewer_qt(config.parent, "GlobeViewer", true, true, true, false, false),
    _globe_generator(std::move(config.globe_generator)) {
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeViewer<PG, NG>::add_elements() {
    add_voronoi_edges();
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeViewer<PG, NG>::add_voronoi_edges() {
    for (const auto &arc : _globe_generator->dual_arcs()) {
        auto source = to_point(arc.source());
        auto target = to_point(arc.target());

        add_circular_arc(source, target);
    }
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeViewer<PG, NG>::add_circular_arc(const Point3 &point1, const Point3 &point2, const CGAL::IO::Color &color) {
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
