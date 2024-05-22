#include "globe/globe_generator.h"

using namespace globe;

int main() {
    const char *filename = "cgal_sphere.ply";

    GlobeGenerator globe_generator;
    globe_generator.generate();
    globe_generator.save_ply(filename);

    return 0;
}
