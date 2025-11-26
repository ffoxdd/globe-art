#ifndef GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_
#define GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_

#include "../../../types.hpp"
#include "../spherical_bounding_box.hpp"
#include <concepts>

namespace globe {

template<typename T>
concept SphericalBoundingBoxSampler = requires(T sampler, const SphericalBoundingBox& bounding_box) {
    { sampler.sample(bounding_box) } -> std::same_as<Point3>;
};

}

#endif //GLOBEART_SRC_GLOBE_GEOMETRY_SPHERICAL_SPHERICAL_BOUNDING_BOX_SAMPLER_HPP_
