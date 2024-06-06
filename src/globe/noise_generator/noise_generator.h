#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_GENERATOR_H_

#include "../types.h"
#include "range.h"
#include <anl/anl.h>

namespace globe {

class NoiseGenerator {
 public:
    template<typename PointRange>
    NoiseGenerator(double low, double high, const PointRange &sample_points) {
        _kernel = std::make_unique<anl::CKernel>();
        _instruction_index = std::make_unique<anl::CInstructionIndex>(initialize_kernel(*_kernel));
        normalize(low, high, sample_points);
    };

    double value(Point3 location);

 private:
    std::unique_ptr<anl::CKernel> _kernel;
    std::unique_ptr<anl::CInstructionIndex> _instruction_index;

    template<typename PointRange>
    void normalize(double low, double high, const PointRange &sample_points);

    static anl::CInstructionIndex initialize_kernel(anl::CKernel &kernel);
    double noise_value(Point3 location);

    Range _noise_range;
    Range _output_range;
};

template<typename PointRange>
void NoiseGenerator::normalize(double low, double high, const PointRange &sample_points) {
    for (const auto &point : sample_points) {
        double sample_value = noise_value(point);
        _noise_range.update_domain(sample_value);
    }

    _output_range = Range(low, high);
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_GENERATOR_H_
