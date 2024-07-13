#include "../globe/types.hpp"
#include "../globe/geometry_viewer/geometry_viewer.hpp"
#include "../globe/globe_generator/spherical_polygon.hpp"
#include "../globe/globe_generator/spherical_bounding_box_sampler.hpp"
#include <QtWidgets/QApplication>
#include <CGAL/Qt/init_ogl_context.h>

using namespace globe;

using Circle3 = SphericalKernel::Circle_3;
using SphericalVector3 = SphericalKernel::Vector_3;

int main(int argc, char *argv[]) {
    CGAL::Qt::init_ogl_context(4, 3);
    QApplication app(argc, argv);
    GeometryViewer geometry_viewer(QApplication::activeWindow());

    Circle3 circle(SphericalPoint3(0, 0, 0), 1.0, SphericalVector3(1, 0, 0));

    SphericalPolygon spherical_polygon(
        std::vector<Arc>{
            Arc(
                circle,
                SphericalPoint3(0.121692, -0.989464, 0.0784364),
                SphericalPoint3(0.119671, -0.98947, 0.0814076)
            ),
            Arc(
                circle,
                SphericalPoint3(0.119671, -0.98947, 0.0814076),
                SphericalPoint3(0.0324765, -0.997948, 0.0551731)
            ),
            Arc(
                circle,
                SphericalPoint3(0.0324765, -0.997948, 0.0551731),
                SphericalPoint3(0.041346, -0.999145, 0.000375392)
            ),
            Arc(
                circle,
                SphericalPoint3(0.041346, -0.999145, 0.000375392),
                SphericalPoint3(0.0892215, -0.995954, -0.0107617)
            ),
            Arc(
                circle,
                SphericalPoint3(0.0892215, -0.995954, -0.0107617),
                SphericalPoint3(0.121692, -0.989464, 0.0784364)
            ),
        }
    );

    auto bounding_box = spherical_polygon.bounding_box();

    SphericalBoundingBoxSampler sampler;

    for (const auto &arc : spherical_polygon.arcs()) {
        auto source = to_point(arc.source());
        auto target = to_point(arc.target());

        geometry_viewer.add_circular_arc(source, target);
    }

    for (int n = 0; n < 1000; n++) {
        Point3 sample_point = sampler.sample(bounding_box);
        bool is_inside = spherical_polygon.contains(sample_point);
        geometry_viewer.add_point(sample_point, is_inside ? BLUE : RED);
    }

    geometry_viewer.show();

    return QApplication::exec();
}
