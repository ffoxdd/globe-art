#include "globe/globe_generator.h"

using namespace globe;

int main() {
    const char *filename = "cgal_sphere.ply";
    GlobeGenerator().generate().save_ply(filename);
    return 0;
}
