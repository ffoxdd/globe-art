#include "globe/globe_generator/globe_generator.hpp"
#include "globe/geometry_viewer/geometry_viewer.hpp"
#include <QtWidgets/QApplication>
#include <CGAL/Qt/init_ogl_context.h>

using namespace globe;

void draw(GlobeGenerator<> &globe_generator, GeometryViewer &geometry_viewer);

int main(int argc, char *argv[]) {
    CGAL::Qt::init_ogl_context(4, 3);
    QApplication app(argc, argv);

    GlobeGenerator<> globe_generator;
    GeometryViewer geometry_viewer(QApplication::activeWindow());

    globe_generator.build();
    draw(globe_generator, geometry_viewer);

//    geometry_viewer.set_key_press_callback([&](GeometryViewer &geometry_viewer, QKeyEvent *event) {
//            globe_generator.relax();
//            draw(globe_generator, geometry_viewer);
//        }
//    );

    geometry_viewer.show();
    return QApplication::exec();
}

bool is_initialized = false;

void draw(GlobeGenerator<> &globe_generator, GeometryViewer &geometry_viewer) {
    geometry_viewer.clear();

    for (const auto &arc : globe_generator.dual_arcs()) {
        geometry_viewer.add_arc(arc);
    }

    if (is_initialized) {
        geometry_viewer.redraw();
    }

    is_initialized = true;
}
