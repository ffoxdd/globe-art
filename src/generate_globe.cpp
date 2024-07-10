#include <QtWidgets/QApplication>
#include <QtWidgets/QOpenGLWidget>
#include <QtGui/QOpenGLFunctions>
#include <CGAL/Qt/init_ogl_context.h>
#include "globe/globe_generator.hpp"
#include "globe/globe_viewer.hpp"

#include <QtWidgets/QWidget>
#include <QtWidgets/QOpenGLWidget>

using namespace globe;

int main(int argc, char *argv[]) {
    CGAL::Qt::init_ogl_context(4, 3);
    QApplication app(argc, argv);

    auto globe_generator = GlobeGenerator();
    globe_generator.generate_points();

    auto globe_viewer = GlobeViewer<>(GlobeViewer<>::Config{
        .parent = QApplication::activeWindow(),
        .globe_generator = std::make_unique<GlobeGenerator<>>(std::move(globe_generator))
    });

    globe_viewer.show();

    return QApplication::exec();
}
