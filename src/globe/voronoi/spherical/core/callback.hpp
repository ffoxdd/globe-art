#ifndef GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_CORE_CALLBACK_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_CORE_CALLBACK_HPP_

#include "sphere.hpp"
#include <functional>

namespace globe::voronoi::spherical {

using Callback = std::function<void(const Sphere&)>;

inline Callback noop_callback() {
    return [](const Sphere&) {};
}

} // namespace globe::voronoi::spherical

#endif //GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_CORE_CALLBACK_HPP_
