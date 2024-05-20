#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/make_mesh_3.h>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <random>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point3;
typedef CGAL::Surface_mesh<Point3> SurfaceMesh;
typedef SurfaceMesh::Property_map<SurfaceMesh::Vertex_index, CGAL::Color> VertexColorMap;

class SphereGenerator {
 public:
    explicit SphereGenerator(
        double radius = 1.0,
        int iterations = 3,
        Point3 center = Point3(0, 0, 0)
    ) : center(center), radius(radius), iterations(iterations) { }

    void generate();
    [[nodiscard]] bool save_ply(const std::string &filename) const;

 private:
    Point3 center;
    double radius;
    int iterations;
    SurfaceMesh mesh;

    void create_icosahedron();
    void subdivide();
    void project_to_sphere();
    void calculate_colors();
};

int main() {
    const double radius = 1.0;
    const int iterations = 3;
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
    calculate_colors();
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
    for (auto vertex : mesh.vertices()) {
        Point3 &point = mesh.point(vertex);
        Kernel::Vector_3 vector = point - CGAL::ORIGIN;

        double scale = radius / std::sqrt(vector.squared_length());
        point = CGAL::ORIGIN + (vector * scale);
    }
}

unsigned char random_value() {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_int_distribution<> dist(0, 255);
    return static_cast<unsigned char>(dist(gen));
}

CGAL::Color random_color() {
    return {random_value(), random_value(), random_value()};
}

void SphereGenerator::calculate_colors() {
    VertexColorMap color_map = mesh.add_property_map<SurfaceMesh::Vertex_index, CGAL::Color>("v:color").first;

    for (auto vertex : mesh.vertices()) {
        color_map[vertex] = random_color();
    }
}
