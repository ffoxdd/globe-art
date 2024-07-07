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

    GlobeViewer viewer(QApplication::activeWindow(), std::move(globe_generator.points_collection()));
    viewer.show();

    return QApplication::exec();
}
