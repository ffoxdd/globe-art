#include "globe/globe_generator/globe_generator.hpp"
#include "globe/geometry_viewer/geometry_viewer.hpp"
#include "globe/noise_generator/constant_scalar_field.hpp"
#include "globe/noise_generator/noise_field.hpp"
#include <QtWidgets/QApplication>
#include <QtCore/Qt>
#include <CGAL/Qt/init_ogl_context.h>
#include <iostream>
#include <utility>
#include <string>
#include <cstring>

using namespace globe;

template<typename GG>
struct DebugState {
    std::vector<typename GG::CellDebugInfo> cells;
    std::size_t current_index = 0;
    bool show_debug = false;
    bool needs_refresh = true;
};

template<typename GG>
void draw(GG &globe_generator, GeometryViewer &geometry_viewer, DebugState<GG> &debug_state);

template<typename GG>
void log_debug_info(const typename GG::CellDebugInfo &info, std::size_t index, std::size_t total);

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
    DebugState<GG> debug_state;

    debug_state.needs_refresh = true;
    draw(globe_generator, geometry_viewer, debug_state);

    geometry_viewer.set_key_press_callback([&](GeometryViewer &viewer, QKeyEvent *event) {
        if (event->key() == Qt::Key_D) {
            debug_state.show_debug = !debug_state.show_debug;
            std::cout << "[Debug] " << (debug_state.show_debug ? "Enabled" : "Disabled")
                << " polygon highlight mode (use Left/Right arrows to cycle)" << std::endl;
            draw(globe_generator, viewer, debug_state);
        } else if (event->key() == Qt::Key_Right || event->key() == Qt::Key_N) {
            if (!debug_state.cells.empty()) {
                debug_state.current_index = (debug_state.current_index + 1) % debug_state.cells.size();
                draw(globe_generator, viewer, debug_state);
            }
        } else if (event->key() == Qt::Key_Left || event->key() == Qt::Key_P) {
            if (!debug_state.cells.empty()) {
                debug_state.current_index =
                    (debug_state.current_index == 0 ? debug_state.cells.size() - 1 : debug_state.current_index - 1);
                draw(globe_generator, viewer, debug_state);
            }
        } else if (event->key() == Qt::Key_R) {
            debug_state.needs_refresh = true;
            draw(globe_generator, viewer, debug_state);
        }
    });

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

bool is_initialized = false;

template<typename GG>
void draw(GG &globe_generator, GeometryViewer &geometry_viewer, DebugState<GG> &debug_state) {
    geometry_viewer.clear();

    for (const auto &arc : globe_generator.dual_arcs()) {
        geometry_viewer.add_arc(arc, CGAL::IO::Color(140, 140, 140));
    }

    if (debug_state.needs_refresh || debug_state.cells.empty()) {
        auto debug_cells = globe_generator.cell_debug_info();
        if (!debug_cells.empty()) {
            debug_state.cells = std::move(debug_cells);
            if (debug_state.current_index >= debug_state.cells.size()) {
                debug_state.current_index = 0;
            }
        } else {
            debug_state.cells.clear();
            debug_state.current_index = 0;
        }
        debug_state.needs_refresh = false;
    }

    if (debug_state.show_debug && !debug_state.cells.empty()) {
        const auto &cell = debug_state.cells[debug_state.current_index];
        for (const auto &arc : cell.dual_cell_arcs) {
            geometry_viewer.add_arc(arc, RED);
            geometry_viewer.add_point(to_point(arc.source()), RED);
            geometry_viewer.add_point(to_point(arc.target()), RED);
            geometry_viewer.add_segment(cell.centroid, to_point(arc.source()), CGAL::IO::Color(200, 0, 0));
        }
        geometry_viewer.add_point(cell.site, BLUE);
        geometry_viewer.add_point(cell.centroid, RED);

        log_debug_info<GG>(cell, debug_state.current_index, debug_state.cells.size());
    }

    if (is_initialized) {
        geometry_viewer.redraw();
    }

    is_initialized = true;
}

template<typename GG>
void log_debug_info(const typename GG::CellDebugInfo &info, std::size_t index, std::size_t total) {
    std::cout << "---- Debugging polygon " << index + 1 << " / " << total << " ----" << std::endl;
    std::cout << "Site: (" << info.site.x() << ", " << info.site.y() << ", " << info.site.z() << ")" << std::endl;
    std::cout << "Mass (Monte Carlo): " << info.mass << std::endl;
    std::cout << "Centroid: (" << info.centroid.x() << ", " << info.centroid.y() << ", " << info.centroid.z() << ")"
              << std::endl;
    std::cout << "Bounding box theta: [" << info.theta_low << ", "
              << info.theta_high << "]" << std::endl;
    std::cout << "Bounding box z: [" << info.z_low << ", "
              << info.z_high << "]" << std::endl;
    std::cout << "Arc count: " << info.dual_cell_arcs.size() << std::endl;
    for (std::size_t i = 0; i < info.dual_cell_arcs.size(); ++i) {
        const auto &arc = info.dual_cell_arcs[i];
        auto source = to_point(arc.source());
        auto target = to_point(arc.target());
        std::cout << "  Arc " << i << ": source(" << source.x() << ", " << source.y() << ", " << source.z() << ")"
                  << " -> target(" << target.x() << ", " << target.y() << ", " << target.z() << ")" << std::endl;
    }
    std::cout << "----------------------------------------" << std::endl;
}
