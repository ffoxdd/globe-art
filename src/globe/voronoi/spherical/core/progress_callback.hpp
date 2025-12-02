#ifndef GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_CORE_PROGRESS_CALLBACK_HPP_
#define GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_CORE_PROGRESS_CALLBACK_HPP_

#include <functional>

namespace globe::voronoi::spherical {

class Sphere;

using ProgressCallback = std::function<void(const Sphere&)>;

inline ProgressCallback no_progress_callback() {
    return [](const Sphere&) {};
}

} // namespace globe::voronoi::spherical

#endif //GLOBEART_SRC_GLOBE_VORONOI_SPHERICAL_CORE_PROGRESS_CALLBACK_HPP_
