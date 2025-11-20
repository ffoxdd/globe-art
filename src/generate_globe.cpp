#include "globe/globe_generator/globe_generator.hpp"
#include "globe/io/qt/voronoi_sphere_qt_renderer.hpp"
#include "globe/scalar_field/constant_scalar_field.hpp"
#include "globe/scalar_field/noise_field.hpp"
#include "globe/integrable_field/density_sampled_integrable_field.hpp"
#include "globe/scalar_field/interval.hpp"
#include "globe/point_generator/random_sphere_point_generator.hpp"
#include "globe/types.hpp"
#include <CLI/CLI.hpp>
#include <CGAL/Qt/init_ogl_context.h>
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <algorithm>

using namespace globe;

int render(const VoronoiSphere &voronoi_sphere, bool render, const std::string &program_name, int argc, char *argv[]);
int run(int points_count, const std::string &density_function, bool render, int optimization_passes, const std::string &program_name, int argc, char *argv[]);

template<typename SF>
int run_with_density_field(int points_count, int optimization_passes, bool render, const std::string &program_name, int argc, char *argv[]);

template<typename SF>
GlobeGenerator<RandomSpherePointGenerator, DensitySampledIntegrableField<SF>> build_globe_generator(int expected_point_count);
std::unique_ptr<QApplication> initialize_q_application(int &argc, char **argv);

int main(int argc, char *argv[]) {
    std::string program_name = argc > 0 ? argv[0] : "generate_globe";

    CLI::App app{"Globe Art Generator"};

    int points_count;
    std::string density_function;
    bool perform_render;
    int optimization_passes;

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

    app.add_option("--optimization-passes", optimization_passes)
        ->description("Number of optimization passes")
        ->default_val(10)
        ->check(CLI::PositiveNumber);

    CLI11_PARSE(app, argc, argv);

    return run(points_count, density_function, perform_render, optimization_passes, program_name, argc, argv);
}

int run(
    int points_count,
    const std::string &density_function,
    bool perform_render,
    int optimization_passes,
    const std::string &program_name,
    int argc, char *argv[]
) {
    std::cout <<
        "Configuration:" << std::endl <<
        "  Points: " << points_count << std::endl <<
        "  Density: " << density_function << std::endl <<
        "  Render: " << (perform_render ? "yes" : "no") << std::endl <<
        "  Optimization passes: " << optimization_passes << std::endl <<
        std::endl;

    if (density_function == "constant") {
        return run_with_density_field<ConstantScalarField>(points_count, optimization_passes, perform_render, program_name, argc, argv);
    } else {
        return run_with_density_field<NoiseField>(points_count, optimization_passes, perform_render, program_name, argc, argv);
    }
}

template<typename SF>
int run_with_density_field(
    int points_count,
    int optimization_passes,
    bool perform_render,
    const std::string &program_name,
    int argc, char *argv[]
) {
    auto globe_generator = build_globe_generator<SF>(points_count);
    VoronoiSphere voronoi_sphere = globe_generator.generate(
        points_count,
        static_cast<size_t>(optimization_passes)
    );

    return render(voronoi_sphere, perform_render, program_name, argc, argv);
}

template<typename SF>
GlobeGenerator<RandomSpherePointGenerator, DensitySampledIntegrableField<SF>> build_globe_generator(int expected_point_count) {
    SF density_field;

    RandomSpherePointGenerator point_generator;
    std::vector<Point3> sample_points;
    for (int i = 0; i < 1000; i++) {
        sample_points.push_back(point_generator.generate());
    }
    density_field.normalize(sample_points);

    size_t target_samples = std::max(
        static_cast<size_t>(60'000),
        static_cast<size_t>(expected_point_count) * 3'000
    );

    auto integrable_field = std::make_unique<DensitySampledIntegrableField<SF>>(
        density_field,
        target_samples
    );

    return GlobeGenerator<RandomSpherePointGenerator, DensitySampledIntegrableField<SF>>(
        RandomSpherePointGenerator(),
        VoronoiSphere(),
        std::move(integrable_field)
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
  VoronoiSphereQtRenderer renderer(nullptr, program_name);
  auto viewer = renderer.render(voronoi_sphere);
  return qt_app->exec();
}
