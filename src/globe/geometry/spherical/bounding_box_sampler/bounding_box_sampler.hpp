#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_

#include "../../../types.hpp"
#include "../bounding_box.hpp"
#include <concepts>

namespace globe::geometry::spherical {

using globe::VectorS2;

template<typename T>
concept BoundingBoxSampler = requires(T sampler, const BoundingBox& bounding_box) {
    { sampler.sample(bounding_box) } -> std::same_as<VectorS2>;
};

} // namespace globe::geometry::spherical

namespace globe {
template<typename T>
concept SphericalBoundingBoxSampler = geometry::spherical::BoundingBoxSampler<T>;
}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_
