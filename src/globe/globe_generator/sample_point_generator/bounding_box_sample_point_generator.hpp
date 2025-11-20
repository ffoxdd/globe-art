#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SAMPLE_POINT_GENERATOR_BOUNDING_BOX_SAMPLE_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SAMPLE_POINT_GENERATOR_BOUNDING_BOX_SAMPLE_POINT_GENERATOR_HPP_

#include "../../types.hpp"
#include "../spherical_bounding_box.hpp"
#include "../spherical_bounding_box_sampler.hpp"

namespace globe {

class BoundingBoxSamplePointGenerator {
 public:
    BoundingBoxSamplePointGenerator() = default;

    BoundingBoxSamplePointGenerator(const BoundingBoxSamplePointGenerator&) = delete;
    BoundingBoxSamplePointGenerator& operator=(const BoundingBoxSamplePointGenerator&) = delete;
    BoundingBoxSamplePointGenerator(BoundingBoxSamplePointGenerator&&) noexcept = default;
    BoundingBoxSamplePointGenerator& operator=(BoundingBoxSamplePointGenerator&&) noexcept = delete;

    Point3 generate(const SphericalBoundingBox &bounding_box);

 private:
    SphericalBoundingBoxSampler _sampler;
};

inline Point3 BoundingBoxSamplePointGenerator::generate(const SphericalBoundingBox &bounding_box) {
    return _sampler.sample(bounding_box);
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SAMPLE_POINT_GENERATOR_BOUNDING_BOX_SAMPLE_POINT_GENERATOR_HPP_
