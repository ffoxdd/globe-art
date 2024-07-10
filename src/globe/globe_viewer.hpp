#ifndef GLOBEART_SRC_GLOBE_GLOBE_VIEWER_H_
#define GLOBEART_SRC_GLOBE_GLOBE_VIEWER_H_

#include "points_collection/points_collection.hpp"
#include "geometry/helpers.hpp"
#include <QtWidgets/QWidget>
#include <CGAL/Qt/Basic_viewer_qt.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Kernel/global_functions_3.h>

#include <iostream>

namespace globe {

const CGAL::IO::Color BLACK(0, 0, 0);
//const CGAL::IO::Color BLUE(0, 0, 255);
//const CGAL::IO::Color RED(255, 0, 0);

template<
    PointGenerator PG = RandomSpherePointGenerator,
    NoiseGenerator NG = AnlNoiseGenerator
>
class GlobeViewer : public CGAL::Basic_viewer_qt {
 public:
    struct Config;

    GlobeViewer();
    explicit GlobeViewer(Config &&config);

    void build();

 protected:
    void build_dual_edges();
    void add_circular_arc(const Point3 &point1, const Point3 &point2, const CGAL::IO::Color &color = BLACK);

 private:
    std::unique_ptr<GlobeGenerator<PG, NG>> _globe_generator{};
    void build_dual_neighborhoods();
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
void GlobeViewer<PG, NG>::build() {
    build_dual_edges();
    build_dual_neighborhoods();
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeViewer<PG, NG>::build_dual_edges() {
    for (const auto &arc : _globe_generator->dual_arcs()) {
        auto source = to_point(arc.source());
        auto target = to_point(arc.target());

        add_circular_arc(source, target);
    }
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeViewer<PG, NG>::build_dual_neighborhoods() {
    for (const auto &dual_neighborhood : _globe_generator->dual_neighborhoods()) {
        std::cout << "point: " << dual_neighborhood.point << std::endl;

        for (const auto &dual_cell_point : dual_neighborhood.dual_cell_points) {
            std::cout << "dual cell: " << dual_cell_point << std::endl;
        }

        std::cout << std::endl;
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
