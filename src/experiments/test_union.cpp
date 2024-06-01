#include "../globe/types.h"
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <iostream>

using namespace globe;

int main() {
    SurfaceMesh mesh1, mesh2, result;

    CGAL::make_icosahedron(mesh1, Point3(0, 0, 0), 1.0);
    CGAL::make_icosahedron(mesh2, Point3(1, 0, 0), 1.0);

//    assert(!CGAL::Polygon_mesh_processing::does_self_intersect(mesh1));
//    assert(!CGAL::Polygon_mesh_processing::does_self_intersect(mesh2));
//    assert(CGAL::Polygon_mesh_processing::does_bound_a_volume(mesh1));
//    assert(CGAL::Polygon_mesh_processing::does_bound_a_volume(mesh2));
//
//    std::cout << "preconditions are met" << std::endl;
//
//    bool is_closed1 = CGAL::is_closed(mesh1);
//    bool is_closed2 = CGAL::is_closed(mesh2);
//    bool is_valid1 = CGAL::is_valid_polygon_mesh(mesh1);
//    bool is_valid2 = CGAL::is_valid_polygon_mesh(mesh2);
//
//    std::cout
//        << "Mesh1 is "
//        << (is_closed1 ? "closed" : "not closed")
//        << " and "
//        << (is_valid1 ? "valid" : "invalid")
//        << std::endl;
//
//    std::cout
//        << "Mesh2 is "
//        << (is_closed2 ? "closed" : "not closed")
//        << " and "
//        << (is_valid2 ? "valid" : "invalid")
//        << std::endl;
//
//    std::cout << "mesh_1 has " << mesh1.vertices().size() << " vertices" << std::endl;
//    std::cout << "mesh_2 has " << mesh2.vertices().size() << " vertices" << std::endl;

    bool valid = CGAL::Polygon_mesh_processing::corefine_and_compute_union(mesh1, mesh2, result);

    std::cout << "valid? " << valid << std::endl;
    std::cout << "result has " << result.vertices().size() << " vertices" << std::endl;

    std::ofstream stream("union.ply");

    if (!stream) {
        return 1;
    }

    bool success = CGAL::IO::write_PLY(stream, result);

    if (!success) {
        return 1;
    }

    return 0
}
