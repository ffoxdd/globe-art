#include "globe/globe_viewer/globe_viewer.hpp"
#include "globe/globe_generator/globe_generator.hpp"
#include <QtWidgets/QApplication>
#include <CGAL/Qt/init_ogl_context.h>

using namespace globe;

int main(int argc, char *argv[]) {
    CGAL::Qt::init_ogl_context(4, 3);
    QApplication app(argc, argv);

    auto viewer = std::make_shared<GeometryViewer>(QApplication::activeWindow());

    GlobeGenerator globe_generator(
        GlobeGenerator<>::Config{
            .relax_callback = [viewer](const RelaxCellIteration &relax_cell_iteration) {
                viewer->add_point(relax_cell_iteration.point, BLUE);
                viewer->add_point(relax_cell_iteration.centroid, RED);
            }
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
