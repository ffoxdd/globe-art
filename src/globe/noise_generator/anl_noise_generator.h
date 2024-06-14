#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_ANL_NOISE_GENERATOR_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_ANL_NOISE_GENERATOR_H_

#include "../types.h"
#include "range.h"
#include "../point_generator/point_generator.h"
#include "../point_generator/random_sphere_point_generator.h"
#include <anl/anl.h>

namespace globe {

const int NORMALIZE_ITERATIONS = 1000;

template<PointGenerator PG = RandomSpherePointGenerator>
class AnlNoiseGenerator {
 public:
    struct Config {
        double low = 0.0; // TODO: see if there's a more direct way of representing [low, high]
        double high = 1.0;

        std::unique_ptr<PG> point_generator =
            std::make_unique<PG>(RandomSpherePointGenerator(1.0));
    };

    AnlNoiseGenerator() : AnlNoiseGenerator(Config()) { };

    explicit AnlNoiseGenerator(Config&& config) :
        _kernel(std::make_unique<anl::CKernel>()),
        _instruction_index(std::make_unique<anl::CInstructionIndex>(initialize_kernel(*_kernel))),
        _point_generator(std::move(config.point_generator)) {
        normalize(config.low, config.high);
    };

    AnlNoiseGenerator(AnlNoiseGenerator&& other) noexcept = default;
    AnlNoiseGenerator& operator=(AnlNoiseGenerator&& other) noexcept = default;

    AnlNoiseGenerator(const AnlNoiseGenerator&) = delete;
    AnlNoiseGenerator& operator=(const AnlNoiseGenerator&) = delete;

    double value(const Point3 &location);

 private:
    std::unique_ptr<PG> _point_generator;
    std::unique_ptr<anl::CKernel> _kernel;
    std::unique_ptr<anl::CInstructionIndex> _instruction_index;

    void normalize(double low, double high);

    static anl::CInstructionIndex initialize_kernel(anl::CKernel &kernel);
    double noise_value(const Point3 &location);

    Range _noise_range;
    Range _output_range;
};

template<PointGenerator PG>
double AnlNoiseGenerator<PG>::value(const Point3 &location) {
    return Range::map(_noise_range, _output_range, noise_value(location));
}

template<PointGenerator PG>
void AnlNoiseGenerator<PG>::normalize(double low, double high) {
    for (int i = 0; i < 1000; i++) {
        double sample_value = noise_value(_point_generator->generate());
        _noise_range.update_domain(sample_value);
    }

    _output_range = Range(low, high);
}

template<PointGenerator PG>
double AnlNoiseGenerator<PG>::noise_value(const Point3 &location) {
    return anl::CNoiseExecutor(*_kernel).evaluateScalar(
        location.x(), location.y(), location.z(), *_instruction_index
    );
}

template<PointGenerator PG>
anl::CInstructionIndex AnlNoiseGenerator<PG>::initialize_kernel(anl::CKernel &kernel) {
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
