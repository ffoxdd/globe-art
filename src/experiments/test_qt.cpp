#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_3.h>
#include <CGAL/draw_triangulation_3.h>
#include <fstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_3<K> Triangulation;
typedef Triangulation::Point Point;

int main(int argc, char **) {
    Triangulation triangulation;

    triangulation.insert(Point(1, 0, 0));
    triangulation.insert(Point(3, 2, 1));
    triangulation.insert(Point(4, 5, 2));
    triangulation.insert(Point(9, 8, 3));
    triangulation.insert(Point(7, 4, 2));
    triangulation.insert(Point(5, 2, 1));
    triangulation.insert(Point(6, 3, 0));
    triangulation.insert(Point(10, 1, 1));

    CGAL::draw(triangulation);

    return EXIT_SUCCESS;
}
