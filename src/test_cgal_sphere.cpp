#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/make_mesh_3.h>
#include <fstream>
#include <iostream>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

class SphereGenerator {
 public:
    SphereGenerator(double radius, int iterations) :
        center(Point_3(0, 0, 0)),
        radius(radius),
        iterations(iterations) { }

    void generate();
    bool save_ply(const std::string &filename) const;

 private:
    Point_3 center;
    double radius;
    int iterations;
    Surface_mesh mesh;
    void create_icosahedron();
    void subdivide();
    void project_to_sphere();
};

int main() {
    const double radius = 1.0;
    const int iterations = 4;
    const char *filename = "cgal_sphere.ply";

    SphereGenerator generator = SphereGenerator(radius, iterations);
    generator.generate();

    if (generator.save_ply(filename)) {
        std::cout << "Successfully exported the sphere" << std::endl;
    } else {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

void SphereGenerator::generate() {
    create_icosahedron();
    subdivide();
    project_to_sphere();
}

void SphereGenerator::create_icosahedron() {
    CGAL::make_icosahedron(mesh, center, radius);
}

void SphereGenerator::subdivide() {
    CGAL::Subdivision_method_3::CatmullClark_subdivision(
        mesh,
        CGAL::parameters::number_of_iterations(iterations)
    );
}

void SphereGenerator::project_to_sphere() {
    for (auto v : mesh.vertices()) {
        Point_3 &p = mesh.point(v);
        Kernel::Vector_3 vector = p - CGAL::ORIGIN;

        double scale = radius / std::sqrt(vector.squared_length());
        p = CGAL::ORIGIN + (vector * scale);
    }
}

bool SphereGenerator::save_ply(const std::string &filename) const {
    std::ofstream os(filename);

    if (!os) {
        std::cerr << "Error: cannot open file " << filename << " for writing." << std::endl;
        return false;
    }

    bool success = CGAL::IO::write_PLY(os, mesh);

    if (!success) {
        std::cerr << "Error: cannot write to " << filename << "." << std::endl;
    }

    return success;
}