#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SAMPLE_POINT_GENERATOR_BOUNDING_BOX_SAMPLE_POINT_GENERATOR_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SAMPLE_POINT_GENERATOR_BOUNDING_BOX_SAMPLE_POINT_GENERATOR_HPP_

#include "../../types.hpp"
#include "sample_point_generator.hpp"
#include "../spherical_bounding_box.hpp"
#include "../spherical_bounding_box_sampler.hpp"

namespace globe {

class BoundingBoxSamplePointGenerator {
 public:
    explicit BoundingBoxSamplePointGenerator(const SphericalBoundingBox &bounding_box);

    BoundingBoxSamplePointGenerator(BoundingBoxSamplePointGenerator &&other) noexcept = default;
    BoundingBoxSamplePointGenerator& operator=(BoundingBoxSamplePointGenerator &&other) noexcept = default;

    BoundingBoxSamplePointGenerator(const BoundingBoxSamplePointGenerator&) = delete;
    BoundingBoxSamplePointGenerator& operator=(const BoundingBoxSamplePointGenerator&) = delete;

    Point3 generate();

 private:
    SphericalBoundingBox _bounding_box;
    SphericalBoundingBoxSampler _sampler;
};

inline BoundingBoxSamplePointGenerator::BoundingBoxSamplePointGenerator(
    const SphericalBoundingBox &bounding_box) :
    _bounding_box(bounding_box),
    _sampler() {
}

inline Point3 BoundingBoxSamplePointGenerator::generate() {
    return _sampler.sample(_bounding_box);
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_SAMPLE_POINT_GENERATOR_BOUNDING_BOX_SAMPLE_POINT_GENERATOR_HPP_
