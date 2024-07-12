#include <QtWidgets/QApplication>
#include <CGAL/Qt/init_ogl_context.h>
#include "globe/points_collection/points_collection.hpp"
#include "globe/globe_generator.hpp"
#include "globe/globe_viewer.hpp"

using namespace globe;

int main(int argc, char *argv[]) {
    CGAL::Qt::init_ogl_context(4, 3);
    QApplication app(argc, argv);

    auto viewer = std::make_shared<GeometryViewer>(QApplication::activeWindow());

    GlobeGenerator globe_generator(GlobeGenerator<>::Config{
            .points_collection = std::make_unique<PointsCollection>(PointsCollection::Config{
                    .dual_neighborhood_callback = [viewer](const DualNeighborhood &dual_neighborhood) {
                        viewer->add_point(dual_neighborhood.point, RED);

                        for (const auto &point : dual_neighborhood.dual_cell_points) {
                            viewer->add_point(point, BLUE);
                        }
                    }
                }
            )
        }
    );

    globe_generator.build();

    GlobeViewer<> globe_viewer(viewer,
        GlobeViewer<>::Config{
            .globe_generator = std::make_unique<GlobeGenerator<>>(std::move(globe_generator))
        }
    );

    globe_viewer.build();
    globe_viewer.show();

    return QApplication::exec();
}
