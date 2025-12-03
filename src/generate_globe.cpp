#include "globe/voronoi/spherical/factories/factory.hpp"
#include "globe/voronoi/spherical/core/progress_callback.hpp"
#include "globe/io/qt/application.hpp"
#include "globe/io/qt/voronoi_sphere_drawer.hpp"
#include <CLI/CLI.hpp>
#include <iostream>
#include <string>
#include <memory>

using namespace globe;
using io::qt::Application;
using io::qt::SphereDrawer;
using voronoi::spherical::Factory;
using voronoi::spherical::Sphere;
using voronoi::spherical::ProgressCallback;
using voronoi::spherical::no_progress_callback;

struct Config {
    int points_count;
    std::string density_field;
    std::string optimization_strategy;
    bool perform_render;
    std::string render_mode;
    int optimization_passes;
    int lloyd_passes;
    int max_perturbations;
};

Config parse_arguments(int argc, char *argv[]);

int main(int argc, char *argv[]) {
    Config config = parse_arguments(argc, argv);

    std::cout <<
        "Configuration:" << std::endl <<
        "  Points: " << config.points_count << std::endl <<
        "  Density: " << config.density_field << std::endl <<
        "  Optimization strategy: " << config.optimization_strategy << std::endl <<
        "  Render: " << (config.perform_render ? "yes" : "no") << std::endl <<
        "  Render mode: " << config.render_mode << std::endl <<
        "  Optimization passes: " << config.optimization_passes << std::endl <<
        "  Lloyd passes: " << config.lloyd_passes << std::endl <<
        "  Max perturbations: " << config.max_perturbations << std::endl <<
        std::endl;

    std::unique_ptr<Application> application;
    std::unique_ptr<SphereDrawer> drawer;
    ProgressCallback progress_callback = no_progress_callback();

    if (config.perform_render) {
        application = std::make_unique<Application>(argc, argv);

        io::qt::RenderMode render_mode = io::qt::RenderMode::Wireframe;
        if (config.render_mode == "solid") {
            render_mode = io::qt::RenderMode::Solid;
        } else if (config.render_mode == "minimal") {
            render_mode = io::qt::RenderMode::Minimal;
        }

        drawer = std::make_unique<SphereDrawer>("Globe", render_mode);
        drawer->show();
        application->process_events();

        progress_callback = [&application, &drawer](const Sphere &sphere) {
            drawer->update(sphere);
            application->process_events();
        };
    }

    Factory factory(
        config.points_count,
        config.density_field,
        config.optimization_strategy,
        config.optimization_passes,
        config.lloyd_passes,
        config.max_perturbations,
        progress_callback
    );

    auto sphere = factory.build();

    if (config.perform_render) {
        drawer->show(*sphere);
        return application->run();
    }

    return 0;
}

Config parse_arguments(int argc, char *argv[]) {
    CLI::App app{"Globe Art Generator"};

    Config config;

    app.add_option("--points,-p", config.points_count)
        ->description("Number of points to generate")
        ->default_val(10);

    app.add_option("--density-field,-f", config.density_field)
        ->description("Density field type")
        ->check(CLI::IsMember({"constant", "linear", "harmonic", "noise"}))
        ->default_val("harmonic");

    app.add_option("--optimization-strategy,-s", config.optimization_strategy)
        ->description("Optimization strategy: ccvd (per-site) or gradient (global)")
        ->check(CLI::IsMember({"ccvd", "gradient"}))
        ->default_val("ccvd");

    app.add_option("--render", config.perform_render)
        ->description("Enable Qt rendering")
        ->default_val(true);

    app.add_option("--render-mode", config.render_mode)
        ->description("Render mode: wireframe, solid, or minimal")
        ->check(CLI::IsMember({"wireframe", "solid", "minimal"}))
        ->default_val("wireframe");

    app.add_option("--optimization-passes", config.optimization_passes)
        ->description("Number of optimization passes")
        ->default_val(100)
        ->check(CLI::PositiveNumber);

    app.add_option("--lloyd-passes", config.lloyd_passes)
        ->description("Number of Lloyd relaxation passes")
        ->default_val(4)
        ->check(CLI::NonNegativeNumber);

    app.add_option("--max-perturbations", config.max_perturbations)
        ->description("Maximum perturbation attempts for gradient optimizer")
        ->default_val(50)
        ->check(CLI::NonNegativeNumber);

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &e) {
        std::exit(app.exit(e));
    }

    return config;
}
