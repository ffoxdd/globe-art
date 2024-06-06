#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/make_mesh_3.h>
#include <fstream>
#include <iostream>
#include <unordered_map>

#define ANL_IMPLEMENTATION
#include <anl/anl.h>

#define STB_IMAGE_IMPLEMENTATION
#include <anl/Imaging/stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <anl/Imaging/stb_image_write.h>

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
    ) : _center(center), _radius(radius), _iterations(iterations) { }

    void generate();
    [[nodiscard]] bool save_ply(const std::string &filename) const;

 private:
    Point3 _center;
    double _radius;
    int _iterations;
    SurfaceMesh _mesh;

    void create_icosahedron();
    void subdivide();
    void project_to_sphere();
    void calculate_colors();

    [[nodiscard]] double noise_value(const Point3 &point) const;
    static double linear_map_value(double value, double low1, double high1, double low2, double high2);
};

int main() {
    const double radius = 1.0;
    const int iterations = 7;
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

    bool success = CGAL::IO::write_PLY(os, _mesh);

    if (!success) {
        std::cerr << "Error: cannot write to " << filename << "." << std::endl;
    }

    return success;
}

void SphereGenerator::create_icosahedron() {
    CGAL::make_icosahedron(_mesh, _center, _radius);
}

void SphereGenerator::subdivide() {
    CGAL::Subdivision_method_3::CatmullClark_subdivision(
        _mesh,
        CGAL::parameters::number_of_iterations(_iterations)
    );
}

void SphereGenerator::project_to_sphere() {
    for (auto vertex : _mesh.vertices()) {
        Point3 &point = _mesh.point(vertex);
        Kernel::Vector_3 vector = point - CGAL::ORIGIN;

        double scale = _radius / std::sqrt(vector.squared_length());
        point = CGAL::ORIGIN + (vector * scale);
    }
}

void SphereGenerator::calculate_colors() {
    VertexColorMap color_map = _mesh.add_property_map<SurfaceMesh::Vertex_index, CGAL::Color>("v:color").first;

    std::vector<double> noise_values(_mesh.num_vertices());
    std::unordered_map<SurfaceMesh::Vertex_index, double> noise_map;

    double min_value = std::numeric_limits<double>::max();
    double max_value = std::numeric_limits<double>::lowest();

    auto points = _mesh.points();

    for (auto vertex : _mesh.vertices()) {
        double value = noise_value(_mesh.point(vertex));
        noise_map[vertex] = value;
        if (value < min_value) min_value = value;
        if (value > max_value) max_value = value;
    }

    for (auto vertex : _mesh.vertices()) {
        double value = noise_map[vertex];
        double mapped_value = linear_map_value(value, min_value, max_value, 0, 255);
        unsigned char color_value = static_cast<int>(mapped_value);

        CGAL::Color color{color_value, color_value, color_value};
        color_map[vertex] = color;
    }
}

double SphereGenerator::noise_value(const Point3 &point) const {
    const double persistence = 1.0;
    const double lacunarity = 2.0;
    const double octaves = 2;
    double frequency = (1 / (2 * _radius));

    anl::CKernel kernel;
    auto seed = kernel.constant(1546);
    anl::CNoiseExecutor executor(kernel);

    auto noise = kernel.fractal(
        seed,
        kernel.simplexBasis(seed),
        kernel.constant(persistence),
        kernel.constant(lacunarity),
        kernel.constant(octaves),
        kernel.constant(frequency)
    );

    noise = kernel.scaleOffset(noise, 1.0 / 32.0, 0.5);
    noise = kernel.gain(kernel.constant(0.95), noise);

    double value = executor.evaluateScalar(point.x(), point.y(), point.z(), noise);
    return value;
}

double SphereGenerator::linear_map_value(double value, double low1, double high1, double low2, double high2) {
    return low2 + (high2 - low2) * ((value - low1) / (high1 - low1));
}

