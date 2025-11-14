#include "globe/globe_generator/globe_generator.hpp"
#include "globe/geometry_viewer/geometry_viewer.hpp"
#include "globe/noise_generator/constant_scalar_field.hpp"
#include <QtWidgets/QApplication>
#include <QtCore/Qt>
#include <CGAL/Qt/init_ogl_context.h>
#include <iostream>
#include <utility>

using namespace globe;

using ConstantGlobeGenerator = GlobeGenerator<RandomSpherePointGenerator, ConstantScalarField>;

struct DebugState {
    std::vector<ConstantGlobeGenerator::CellDebugInfo> cells;
    std::size_t current_index = 0;
    bool show_debug = false;
    bool needs_refresh = true;
};

void draw(ConstantGlobeGenerator &globe_generator, GeometryViewer &geometry_viewer, DebugState &debug_state);
void log_debug_info(const ConstantGlobeGenerator::CellDebugInfo &info, std::size_t index, std::size_t total);

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

    DebugState debug_state;

    globe_generator.build();
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

bool is_initialized = false;

void draw(ConstantGlobeGenerator &globe_generator, GeometryViewer &geometry_viewer, DebugState &debug_state) {
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

        log_debug_info(cell, debug_state.current_index, debug_state.cells.size());
    }

    if (is_initialized) {
        geometry_viewer.redraw();
    }

    is_initialized = true;
}

void log_debug_info(const ConstantGlobeGenerator::CellDebugInfo &info, std::size_t index, std::size_t total) {
    std::cout << "---- Debugging polygon " << index + 1 << " / " << total << " ----" << std::endl;
    std::cout << "Site: (" << info.site.x() << ", " << info.site.y() << ", " << info.site.z() << ")" << std::endl;
    std::cout << "Area (Monte Carlo): " << info.capacity << std::endl;
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
