#include "globe/globe_generator/globe_generator.hpp"
#include "globe/geometry_viewer/geometry_viewer.hpp"
#include "globe/noise_generator/constant_scalar_field.hpp"
#include <QtWidgets/QApplication>
#include <CGAL/Qt/init_ogl_context.h>
#include <iomanip>
#include <sstream>

using namespace globe;

using ConstantGlobeGenerator = GlobeGenerator<RandomSpherePointGenerator, ConstantScalarField>;

void draw(ConstantGlobeGenerator &globe_generator, GeometryViewer &geometry_viewer);

int main(int argc, char *argv[]) {
    CGAL::Qt::init_ogl_context(4, 3);
    QApplication app(argc, argv);

    ConstantScalarField constant_density_field(1.0);
    ConstantGlobeGenerator globe_generator(
        RandomSpherePointGenerator(1.0),
        PointsCollection(),
        constant_density_field
    );

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

void draw(ConstantGlobeGenerator &globe_generator, GeometryViewer &geometry_viewer) {
    geometry_viewer.clear();

    for (const auto &arc : globe_generator.dual_arcs()) {
        geometry_viewer.add_arc(arc);
    }

    auto capacities = globe_generator.cell_capacities();
    for (const auto &[position, capacity] : capacities) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3) << capacity;
        geometry_viewer.add_text(position, oss.str());
    }

    if (is_initialized) {
        geometry_viewer.redraw();
    }

    is_initialized = true;
}
