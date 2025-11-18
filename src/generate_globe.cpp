#include "globe/globe_generator/globe_generator.hpp"
#include "globe/geometry_viewer/geometry_viewer.hpp"
#include "globe/scalar_field/constant_scalar_field.hpp"
#include "globe/scalar_field/noise_field.hpp"
#include <QtWidgets/QApplication>
#include <QtCore/Qt>
#include <CGAL/Qt/init_ogl_context.h>
#include <iostream>
#include <utility>
#include <string>
#include <cstring>

using namespace globe;

template<typename GG>
void draw(GG &globe_generator, GeometryViewer &geometry_viewer);

void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]\n"
        << "Options:\n"
        << "  --points <N>                Number of points to generate (default: 10)\n"
        << "  --density-function <type>   Density function: 'constant' or 'noise' (default: noise)\n"
        << "  --no-render                 Run without Qt rendering (terminal only)\n"
        << "  --help                      Show this help message\n"
        << std::endl;
}

template<typename GG>
int render_globe(GG &globe_generator, int argc, char *argv[], bool render) {
    if (!render) {
        return 0;
    }

    CGAL::Qt::init_ogl_context(4, 3);
    QApplication app(argc, argv);
    GeometryViewer geometry_viewer(QApplication::activeWindow());

    draw(globe_generator, geometry_viewer);

    geometry_viewer.show();
    return QApplication::exec();
}

template<typename GG>
int run_globe_generator(GG &globe_generator, int argc, char *argv[], int num_points, bool render) {
    globe_generator.build(num_points);
    std::cout << "Globe generation complete!" << std::endl;
    return render_globe(globe_generator, argc, argv, render);
}

int main(int argc, char *argv[]) {
    int num_points = 10;
    std::string density_function = "noise";
    bool render = true;

    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--points") == 0 && i + 1 < argc) {
            num_points = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--density-function") == 0 && i + 1 < argc) {
            density_function = argv[++i];
            if (density_function != "constant" && density_function != "noise") {
                std::cerr << "Error: Unknown density function '" << density_function << "'. Use 'constant' or 'noise'." << std::endl;
                return 1;
            }
        } else if (std::strcmp(argv[i], "--no-render") == 0) {
            render = false;
        } else if (std::strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    std::cout << "Configuration:" << std::endl;
    std::cout << "  Points: " << num_points << std::endl;
    std::cout << "  Density: " << density_function << std::endl;
    std::cout << "  Render: " << (render ? "yes" : "no") << std::endl;
    std::cout << std::endl;

    if (density_function == "constant") {
        ConstantScalarField constant_density_field(1.0);
        GlobeGenerator<RandomSpherePointGenerator, ConstantScalarField> globe_generator(
            RandomSpherePointGenerator(1.0),
            PointsCollection(),
            constant_density_field
        );
        return run_globe_generator(globe_generator, argc, argv, num_points, render);
    }

    GlobeGenerator<RandomSpherePointGenerator, NoiseField> globe_generator(
        RandomSpherePointGenerator(1.0),
        PointsCollection(),
        NoiseField()
    );

    return run_globe_generator(globe_generator, argc, argv, num_points, render);
}

template<typename GG>
void draw(GG &globe_generator, GeometryViewer &geometry_viewer) {
    geometry_viewer.clear();

    for (const auto &arc : globe_generator.dual_arcs()) {
        geometry_viewer.add_arc(arc, CGAL::IO::Color(140, 140, 140));
    }
}
