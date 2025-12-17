#include "globe/io/text/sphere_repository.hpp"
#include "globe/io/ply/exporter.hpp"
#include <CLI/CLI.hpp>
#include <filesystem>
#include <iostream>
#include <string>

using globe::io::text::SphereRepository;
using globe::io::ply::Exporter;

int main(int argc, char* argv[]) {
    CLI::App app{"Convert sphere text file to PLY mesh"};

    std::string input_file;
    double arc_thickness = 0.02;
    double max_edge_length = 0.05;

    app.add_option("input", input_file)
        ->description("Input sphere text file")
        ->required();

    app.add_option("--thickness,-t", arc_thickness)
        ->description("Arc bar thickness")
        ->default_val(0.02);

    app.add_option("--resolution,-r", max_edge_length)
        ->description("Maximum edge length for arc subdivision")
        ->default_val(0.05);

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }

    std::filesystem::path input_path(input_file);

    if (!std::filesystem::exists(input_path)) {
        std::cerr << "File not found: " << input_path << std::endl;
        return 1;
    }

    auto sphere = SphereRepository::load(input_path.string());

    std::filesystem::path output_path = input_path;
    output_path.replace_extension(".ply");

    std::cout <<
        "Configuration:" << std::endl <<
        "  Input: " << input_path << std::endl <<
        "  Output: " << output_path << std::endl <<
        "  Thickness: " << arc_thickness << std::endl <<
        "  Resolution: " << max_edge_length << std::endl <<
        std::endl;

    Exporter::save_ply(*sphere, output_path.string(), arc_thickness, max_edge_length);

    std::cout << "Saved: " << output_path << std::endl;

    return 0;
}
