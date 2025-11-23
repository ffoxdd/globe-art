#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_FIELD_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_FIELD_H_

#include "../../types.hpp"
#include "../../math/interval.hpp"
#include <anl/anl.h>
#include <cmath>

namespace globe {

class NoiseField {
 public:
    NoiseField(Interval output_range = Interval(0, 1), int seed = 1546);
    double value(const Point3 &location);
    Interval output_range() const { return _output_range; }

 private:
    anl::CKernel _kernel;
    anl::CInstructionIndex _instruction_index;
    Interval _output_range;
    int _seed;

    static constexpr double NOISE_FREQUENCY = 1.0;
    static constexpr int NOISE_OCTAVES = 2;
    static constexpr double NOISE_PERSISTENCE = 0.5;
    static constexpr double NOISE_LACUNARITY = 2.0;
    static constexpr double CONTRAST_GAIN = 0.95;

    static double amplitude_sum(double persistence, int octaves);
    static double fractal_range_scale(double persistence, int octaves);
    static anl::CInstructionIndex initialize_kernel(anl::CKernel &kernel, Interval output_range, int seed);
};

inline NoiseField::NoiseField(Interval output_range, int seed) :
    _kernel(),
    _instruction_index(initialize_kernel(_kernel, output_range, seed)),
    _output_range(output_range),
    _seed(seed) {
}

inline double NoiseField::value(const Point3 &location) {
    return anl::CNoiseExecutor(_kernel).evaluateScalar(
        location.x(), location.y(), location.z(), _instruction_index
    );
}

inline double NoiseField::amplitude_sum(double persistence, int octaves) {
    double sum = 0.0;

    for (int i = 0; i < octaves; ++i) {
        sum += std::pow(persistence, i);
    }

    return sum;
}

inline double NoiseField::fractal_range_scale(double persistence, int octaves) {
    constexpr double SIMPLEX_EMPIRICAL_FACTOR = 10.5;

    double amplitude = amplitude_sum(persistence, octaves);
    double range = 2.0 * SIMPLEX_EMPIRICAL_FACTOR * amplitude;

    return 1.0 / range;
}

inline anl::CInstructionIndex NoiseField::initialize_kernel(anl::CKernel &kernel, Interval output_range, int seed) {
    auto seed_instruction = kernel.constant(seed);

    auto instruction_index = kernel.fractal(
        seed_instruction,
        kernel.simplexBasis(seed_instruction),
        kernel.constant(NOISE_PERSISTENCE),
        kernel.constant(NOISE_LACUNARITY),
        kernel.constant(NOISE_OCTAVES),
        kernel.constant(NOISE_FREQUENCY)
    );

    double scale = fractal_range_scale(NOISE_PERSISTENCE, NOISE_OCTAVES);
    instruction_index = kernel.scaleOffset(instruction_index, scale, 0.5);
    instruction_index = kernel.gain(kernel.constant(CONTRAST_GAIN), instruction_index);

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

