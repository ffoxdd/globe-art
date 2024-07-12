#ifndef GLOBEART_SRC_GLOBE_GLOBE_VIEWER_H_
#define GLOBEART_SRC_GLOBE_GLOBE_VIEWER_H_

#include "globe_generator.hpp"
#include "geometry_viewer/geometry_viewer.hpp"
#include "geometry/helpers.hpp"
#include <CGAL/IO/Color.h>
#include <CGAL/Kernel/global_functions_3.h>
#include <QtWidgets/QWidget>

namespace globe {

template<
    PointGenerator PG = RandomSpherePointGenerator,
    NoiseGenerator NG = AnlNoiseGenerator
>
class GlobeViewer {
 public:
    struct Config;

    explicit GlobeViewer(QWidget *parent);
    GlobeViewer(QWidget *parent, Config &&config);
    explicit GlobeViewer(std::shared_ptr<GeometryViewer> viewer, Config &&config);

    void build();
    void show();

 private:
    std::shared_ptr<GeometryViewer> _geometry_viewer;
    std::unique_ptr<GlobeGenerator<PG, NG>> _globe_generator;

    void build_dual_edges();
    void build_dual_neighborhoods();
};

template<PointGenerator PG, NoiseGenerator NG>
struct GlobeViewer<PG, NG>::Config {
    std::unique_ptr<GlobeGenerator<PG, NG>> globe_generator = std::make_unique<GlobeGenerator<PG, NG>>();
};

template<PointGenerator PG, NoiseGenerator NG>
GlobeViewer<PG, NG>::GlobeViewer(QWidget *parent) :
    GlobeViewer<PG, NG>::GlobeViewer(parent, Config()) {
}

template<PointGenerator PG, NoiseGenerator NG>
GlobeViewer<PG, NG>::GlobeViewer(QWidget *parent, GlobeViewer::Config &&config) :
    GlobeViewer<PG, NG>::GlobeViewer(
        std::make_shared<GeometryViewer>(GeometryViewer(parent)),
        std::move(config)
    ) {
}

template<PointGenerator PG, NoiseGenerator NG>
GlobeViewer<PG, NG>::GlobeViewer(std::shared_ptr<GeometryViewer> geometry_viewer, GlobeViewer::Config &&config) :
    _geometry_viewer(std::move(geometry_viewer)),
    _globe_generator(std::move(config.globe_generator)) {
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeViewer<PG, NG>::build() {
//    build_dual_edges();
    build_dual_neighborhoods();
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeViewer<PG, NG>::show() {
    _geometry_viewer->show();
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeViewer<PG, NG>::build_dual_edges() {
    for (const auto &arc : _globe_generator->dual_arcs()) {
        auto source = to_point(arc.source());
        auto target = to_point(arc.target());

        _geometry_viewer->add_circular_arc(source, target);
    }
}

template<PointGenerator PG, NoiseGenerator NG>
void GlobeViewer<PG, NG>::build_dual_neighborhoods() {
    for (const auto &dual_neighborhood : _globe_generator->dual_neighborhoods()) {
        auto cell_points = dual_neighborhood.dual_cell_points;

        for (size_t i = 0; i < cell_points.size(); ++i) {
            size_t i_ = (i + 1) % cell_points.size();

            auto source = cell_points[i];
            auto target = cell_points[i_];

            _geometry_viewer->add_circular_arc(source, target, BLUE);
        }
    }
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_VIEWER_H_
