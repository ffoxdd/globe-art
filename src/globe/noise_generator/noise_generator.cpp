#include "noise_generator.h"

namespace globe {

NoiseGenerator::NoiseGenerator() {
    _kernel = std::make_unique<anl::CKernel>();
    _instruction_index = std::make_unique<anl::CInstructionIndex>(initialize_kernel(*_kernel));
}

double NoiseGenerator::value(Point3 location) {
    return anl::CNoiseExecutor(*_kernel).evaluateScalar(
        location.x(), location.y(), location.z(), *_instruction_index
    );
}

template<typename SamplePointsIterator>
void NoiseGenerator::normalize(double min, double max, SamplePointsIterator begin, SamplePointsIterator end) {

}

anl::CInstructionIndex NoiseGenerator::initialize_kernel(anl::CKernel &kernel) {
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
