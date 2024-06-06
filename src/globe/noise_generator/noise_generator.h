#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_GENERATOR_H_

#include "../types.h"
#include <anl/anl.h>

namespace globe {

class NoiseGenerator {
 public:
    NoiseGenerator();
    double value(Point3 location);

    template<typename SamplePointsIterator>
    void normalize(double min, double max, SamplePointsIterator begin, SamplePointsIterator end);

 private:
    std::unique_ptr<anl::CKernel> _kernel;
    std::unique_ptr<anl::CInstructionIndex> _instruction_index;
    static anl::CInstructionIndex initialize_kernel(anl::CKernel &kernel);
};

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_GENERATOR_H_
