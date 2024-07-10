#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_ANL_NOISE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_ANL_NOISE_GENERATOR_H_

#include "../types.hpp"
#include "range.hpp"
#include "../point_generator/point_generator.hpp"
#include "../point_generator/random_sphere_point_generator.hpp"
#include <anl/anl.h>

namespace globe {

class AnlNoiseGenerator {
 public:
    AnlNoiseGenerator();

    void normalize(const std::vector<Point3> &sample_points, Range output_range);
    double value(const Point3 &location);

 private:
    std::unique_ptr<anl::CKernel> _kernel;
    std::unique_ptr<anl::CInstructionIndex> _instruction_index;

    static anl::CInstructionIndex initialize_kernel(anl::CKernel &kernel);
    double noise_value(const Point3 &location);

    Range _noise_range;
    Range _output_range;
};

AnlNoiseGenerator::AnlNoiseGenerator() :
    _kernel(std::make_unique<anl::CKernel>()),
    _instruction_index(std::make_unique<anl::CInstructionIndex>(initialize_kernel(*_kernel))),
    _noise_range(Range(0.0, 1.0)),
    _output_range(Range(0.0, 1.0)) {
}

void AnlNoiseGenerator::normalize(const std::vector<Point3> &sample_points, Range output_range) {
    for (const auto &point : sample_points) {
        double sample_value = noise_value(point);
        _noise_range.update_domain(sample_value);
    }

    _output_range = output_range;
}

double AnlNoiseGenerator::value(const Point3 &location) {
    return Range::map(_noise_range, _output_range, noise_value(location));
}

double AnlNoiseGenerator::noise_value(const Point3 &location) {
    return anl::CNoiseExecutor(*_kernel).evaluateScalar(
        location.x(), location.y(), location.z(), *_instruction_index
    );
}

anl::CInstructionIndex AnlNoiseGenerator::initialize_kernel(anl::CKernel &kernel) {
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

    return instruction_index;
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_ANL_NOISE_GENERATOR_H_
