#include <type_traits>
#include "../globe/types.h"
#include <iostream>

using namespace globe;

int main() {
    static_assert(std::is_move_constructible<SurfaceMesh>::value, "SurfaceMesh is not move constructible!");
    static_assert(std::is_move_assignable<SurfaceMesh>::value, "SurfaceMesh is not move assignable!");

    std::cout << "ok!" << std::endl;
}



