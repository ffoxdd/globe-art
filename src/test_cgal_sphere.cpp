#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/make_mesh_3.h>
#include <fstream>
#include <iostream>
#include <noise/noise.h>

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

    CGAL::Color noise_color(const Point3 &point);
    [[nodiscard]] double noise_value(const Point3 &point) const;
    static double linear_map_value(double value, double low1, double high1, double low2, double high2);
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

void SphereGenerator::calculate_colors() {
    VertexColorMap color_map = mesh.add_property_map<SurfaceMesh::Vertex_index, CGAL::Color>("v:color").first;

    for (auto vertex : mesh.vertices()) {
        Point3 &point = mesh.point(vertex);
        color_map[vertex] = noise_color(point);
    }
}

CGAL::Color SphereGenerator::noise_color(const Point3 &point) {
    double value = noise_value(point);
    unsigned char color_value = static_cast<int>(linear_map_value(value, -1, 1, 0, 255));
    return {color_value, color_value, color_value};
}

double SphereGenerator::noise_value(const Point3 &point) const {
    noise::module::Perlin perlin_noise;

    perlin_noise.SetSeed(1278269);
    perlin_noise.SetFrequency((1 / (radius * 2)) * 1.95);
    perlin_noise.SetLacunarity(1.5);
    perlin_noise.SetOctaveCount(2);
    perlin_noise.SetNoiseQuality(noise::QUALITY_STD);

    return perlin_noise.GetValue(point.x(), point.y(), point.z());
}

double SphereGenerator::linear_map_value(double value, double low1, double high1, double low2, double high2) {
    return low2 + (high2 - low2) * ((value - low1) / (high1 - low1));
}

