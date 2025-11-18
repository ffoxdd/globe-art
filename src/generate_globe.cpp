#include "globe/globe_generator/globe_generator.hpp"
#include "globe/geometry_viewer/geometry_viewer.hpp"
#include "globe/scalar_field/constant_scalar_field.hpp"
#include "globe/scalar_field/noise_field.hpp"
#include <QtWidgets/QApplication>
#include <QtCore/Qt>
#include <CGAL/Qt/init_ogl_context.h>
#include <CLI/CLI.hpp>
#include <iostream>
#include <string>

using namespace globe;

int run(const char *program_name, int points_count, const std::string &density_function, bool render);

template<typename ScalarFieldType>
GlobeGenerator<RandomSpherePointGenerator, ScalarFieldType> build_globe_generator();

template<typename ScalarFieldType>
int run_with_density_field(int points_count, bool render, const char *program_name);

QApplication build_application(const char *program_name);

template<typename GG>
int render_qt(GG &globe_generator, bool render, const char *program_name);

template<typename GG>
void draw(GG &globe_generator, GeometryViewer &geometry_viewer);

int main(int argc, char *argv[]) {
    CLI::App app{"Globe Art Generator"};

    int points_count;
    std::string density_function;
    bool render;

    app.add_option(
        "-p,--points",
        points_count,
        "Number of points to generate"
    )
    ->default_val(10)
    ->required();

    app.add_option(
        "-d,--density-function",
        density_function,
        "Density function type"
    )
    ->check(CLI::IsMember({"constant", "noise"}))
    ->default_val("noise");

    app.add_option(
        "--render",
        render,
        "Enable Qt rendering"
    )
    ->default_val(true);

    CLI11_PARSE(app, argc, argv);

    const char *program_name = argc > 0 ? argv[0] : "generate_globe";
    return run(program_name, points_count, density_function, render);
}

int run(const char *program_name, int points_count, const std::string &density_function, bool render) {
    std::cout <<
        "Configuration:" << std::endl <<
        "  Points: " << points_count << std::endl <<
        "  Density: " << density_function << std::endl <<
        "  Render: " << (render ? "yes" : "no") << std::endl <<
        std::endl;

    if (density_function == "constant") {
        return run_with_density_field<ConstantScalarField>(points_count, render, program_name);
    } else {
        return run_with_density_field<NoiseField>(points_count, render, program_name);
    }
}

template<typename SF>
int run_with_density_field(int points_count, bool render, const char *program_name) {
    auto globe_generator = build_globe_generator<SF>();
    globe_generator.generate(points_count);
    return render_qt(globe_generator, render, program_name);
}

template<typename SF>
GlobeGenerator<RandomSpherePointGenerator, SF> build_globe_generator() {
    return GlobeGenerator<RandomSpherePointGenerator, SF>(
        RandomSpherePointGenerator(),
        PointsCollection(),
        SF()
    );
}

QApplication build_application(const char *program_name) {
    CGAL::Qt::init_ogl_context(4, 3);
    int qt_argc = 1;
    char *qt_argv[] = { const_cast<char*>(program_name), nullptr };
    return QApplication(qt_argc, qt_argv);
}

template<typename GG>
int render_qt(GG &globe_generator, bool render, const char *program_name) {
    if (!render) {
        return 0;
    }

    QApplication app = build_application(program_name);
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
