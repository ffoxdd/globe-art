#include "globe/globe_generator/globe_generator.hpp"
#include "globe/io/qt/voronoi_sphere_qt_renderer.hpp"
#include "globe/scalar_field/constant_scalar_field.hpp"
#include "globe/scalar_field/noise_field.hpp"
#include <CLI/CLI.hpp>
#include <CGAL/Qt/init_ogl_context.h>
#include <iostream>
#include <string>
#include <memory>

using namespace globe;

int render(const VoronoiSphere &voronoi_sphere, bool render, const std::string &program_name, int argc, char *argv[]);
int run(int points_count, const std::string &density_function, bool render, const std::string &program_name, int argc, char *argv[]);

template<typename SF>
int run_with_density_field(int points_count, bool render, const std::string &program_name, int argc, char *argv[]);

template<typename SF>
GlobeGenerator<RandomSpherePointGenerator, SF> build_globe_generator();
std::unique_ptr<QApplication> initialize_q_application(int &argc, char **argv);

int main(int argc, char *argv[]) {
    std::string program_name = argc > 0 ? argv[0] : "generate_globe";

    CLI::App app{"Globe Art Generator"};

    int points_count;
    std::string density_function;
    bool perform_render;

    app.add_option("--points,-p", points_count)
        ->description("Number of points to generate")
        ->default_val(10);

    app.add_option("--density-function,-d", density_function)
        ->description("Density function type")
        ->check(CLI::IsMember({"constant", "noise"}))
        ->default_val("noise");

    app.add_option("--render", perform_render)
        ->description("Enable Qt rendering")
        ->default_val(true);

    CLI11_PARSE(app, argc, argv);

    return run(points_count, density_function, perform_render, program_name, argc, argv);
}

int run(
    int points_count,
    const std::string &density_function,
    bool perform_render,
    const std::string &program_name,
    int argc, char *argv[]
) {
    std::cout <<
        "Configuration:" << std::endl <<
        "  Points: " << points_count << std::endl <<
        "  Density: " << density_function << std::endl <<
        "  Render: " << (perform_render ? "yes" : "no") << std::endl <<
        std::endl;

    if (density_function == "constant") {
        return run_with_density_field<ConstantScalarField>(points_count, perform_render, program_name, argc, argv);
    } else {
        return run_with_density_field<NoiseField>(points_count, perform_render, program_name, argc, argv);
    }
}

template<typename SF>
int run_with_density_field(
    int points_count,
    bool perform_render,
    const std::string &program_name,
    int argc, char *argv[]
) {
    auto globe_generator = build_globe_generator<SF>();
    VoronoiSphere voronoi_sphere = globe_generator.generate(points_count);

    return render(voronoi_sphere, perform_render, program_name, argc, argv);
}

template<typename SF>
GlobeGenerator<RandomSpherePointGenerator, SF> build_globe_generator() {
    return GlobeGenerator<RandomSpherePointGenerator, SF>(
        RandomSpherePointGenerator(),
        VoronoiSphere(),
        SF()
    );
}

std::unique_ptr<QApplication> initialize_q_application(int &argc, char **argv) {
  CGAL::Qt::init_ogl_context(4, 3);
  return std::make_unique<QApplication>(argc, argv);
}

int render(
    const VoronoiSphere &voronoi_sphere,
    bool render,
    const std::string &program_name,
    int argc,
    char *argv[]
) {
  if (!render) {
    return 0;
  }

  auto qt_app = initialize_q_application(argc, argv);
  VoronoiSphereQtRenderer renderer(QApplication::activeWindow(), program_name);
  renderer.render(voronoi_sphere);
  return qt_app->exec();
}
