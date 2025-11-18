#include "globe/globe_generator/globe_generator.hpp"
#include "globe/geometry_viewer/geometry_viewer.hpp"
#include "globe/scalar_field/constant_scalar_field.hpp"
#include "globe/scalar_field/noise_field.hpp"
#include <QtWidgets/QApplication>
#include <QtCore/Qt>
#include <CGAL/Qt/init_ogl_context.h>
#include <iostream>
#include <string>
#include <cstring>

using namespace globe;

void print_usage(const char *program_name);

int execute(const char *program_name, int points_count, const std::string &density_function, bool render);

template<typename ScalarFieldType>
GlobeGenerator<RandomSpherePointGenerator, ScalarFieldType> create_globe_generator();

template<typename ScalarFieldType>
int run_with_density_field(int points_count, bool render, const char *program_name);

template<typename GG>
void build(GG &globe_generator, int points_count);

template<typename GG>
int run(GG &globe_generator, int points_count, bool render, const char *program_name);

template<typename GG>
int render_qt(GG &globe_generator, bool render, const char *program_name);

template<typename GG>
void draw(GG &globe_generator, GeometryViewer &geometry_viewer);

int main(int argc, char *argv[]) {
    const char *program_name = argc > 0 ? argv[0] : "generate_globe";
    int points_count = 10;
    std::string density_function = "noise";
    bool render = true;

    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--points") == 0 && i + 1 < argc) {
            points_count = std::stoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--density-function") == 0 && i + 1 < argc) {
            density_function = argv[++i];
            if (density_function != "constant" && density_function != "noise") {
                std::cerr << "Error: Unknown density function '" << density_function << "'. Use 'constant' or 'noise'." << std::endl;
                return 1;
            }
        } else if (std::strcmp(argv[i], "--no-render") == 0) {
            render = false;
        } else if (std::strcmp(argv[i], "--help") == 0) {
            print_usage(program_name);
            return 0;
        }
    }

    return execute(program_name, points_count, density_function, render);
}

template<typename SF>
GlobeGenerator<RandomSpherePointGenerator, SF> create_globe_generator() {
    return GlobeGenerator<RandomSpherePointGenerator, SF>(
        RandomSpherePointGenerator(),
        PointsCollection(),
        SF()
    );
}

template<typename SF>
int run_with_density_field(int points_count, bool render, const char *program_name) {
    auto globe_generator = create_globe_generator<SF>();
    return run(globe_generator, points_count, render, program_name);
}

int execute(const char *program_name, int points_count, const std::string &density_function, bool render) {
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Points: " << points_count << std::endl;
    std::cout << "  Density: " << density_function << std::endl;
    std::cout << "  Render: " << (render ? "yes" : "no") << std::endl;
    std::cout << std::endl;

    if (density_function == "constant") {
        return run_with_density_field<ConstantScalarField>(points_count, render, program_name);
    }

    return run_with_density_field<NoiseField>(points_count, render, program_name);
}

void print_usage(const char *program_name) {
    std::cout << "Usage: " << program_name << " [options]\n"
        << "Options:\n"
        << "  --points <N>                Number of points to generate (default: 10)\n"
        << "  --density-function <type>   Density function: 'constant' or 'noise' (default: noise)\n"
        << "  --no-render                 Run without Qt rendering (terminal only)\n"
        << "  --help                      Show this help message\n"
        << std::endl;
}

template<typename GG>
int run(GG &globe_generator, int points_count, bool render, const char *program_name) {
    build(globe_generator, points_count);
    return render_qt(globe_generator, render, program_name);
}

template<typename GG>
void build(GG &globe_generator, int points_count) {
    globe_generator.build(points_count);
    std::cout << "Globe generation complete!" << std::endl;
}

template<typename GG>
int render_qt(GG &globe_generator, bool render, const char *program_name) {
    if (!render) {
        return 0;
    }

    CGAL::Qt::init_ogl_context(4, 3);
    int qt_argc = 1;
    char *qt_argv[] = { const_cast<char*>(program_name), nullptr };
    QApplication app(qt_argc, qt_argv);

    GeometryViewer geometry_viewer(QApplication::activeWindow());

    draw(globe_generator, geometry_viewer);

    geometry_viewer.show();
    return QApplication::exec();
}

template<typename GG>
void draw(GG &globe_generator, GeometryViewer &geometry_viewer) {
    geometry_viewer.clear();

    for (const auto &arc : globe_generator.dual_arcs()) {
        geometry_viewer.add_arc(arc, CGAL::IO::Color(140, 140, 140));
    }
}
