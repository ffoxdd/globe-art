#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_FIELD_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_FIELD_H_

#include "../../types.hpp"
#include "../../math/interval.hpp"
#include <anl/anl.h>

namespace globe {

class NoiseField {
 public:
    NoiseField(Interval output_range = Interval(0, 1));
    double value(const Point3 &location);
    Interval output_range() const { return _output_range; }

 private:
    anl::CKernel _kernel;
    anl::CInstructionIndex _instruction_index;
    Interval _output_range;

    static anl::CInstructionIndex initialize_kernel(anl::CKernel &kernel, Interval output_range);
};

inline NoiseField::NoiseField(Interval output_range) :
    _kernel(),
    _instruction_index(initialize_kernel(_kernel, output_range)),
    _output_range(output_range) {
}

inline double NoiseField::value(const Point3 &location) {
    return anl::CNoiseExecutor(_kernel).evaluateScalar(
        location.x(), location.y(), location.z(), _instruction_index
    );
}

inline anl::CInstructionIndex NoiseField::initialize_kernel(anl::CKernel &kernel, Interval output_range) {
    const double persistence = 0.5;
    const double lacunarity = 2.0;
    const double octaves = 2;
    const double frequency = 1.0;

    auto seed = kernel.constant(1546);

    auto instruction_index = kernel.fractal(
        seed,
        kernel.simplexBasis(seed),
        kernel.constant(persistence),
        kernel.constant(lacunarity),
        kernel.constant(octaves),
        kernel.constant(frequency)
    );

    instruction_index = kernel.scaleOffset(instruction_index, 1.0 / 32.0, 0.5);
    instruction_index = kernel.gain(kernel.constant(0.95), instruction_index);

    double range_scale = output_range.measure();
    double range_offset = output_range.low();
    instruction_index = kernel.scaleOffset(instruction_index, range_scale, range_offset);

    instruction_index = kernel.clamp(
        instruction_index,
        kernel.constant(output_range.low()),
        kernel.constant(output_range.high())
    );

    return instruction_index;
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_FIELD_H_

