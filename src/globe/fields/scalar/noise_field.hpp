#ifndef GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_FIELD_H_
#define GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_FIELD_H_

#include "../../types.hpp"
#include "../../math/interval.hpp"
#include <anl/anl.h>
#include <memory>
#include <ranges>
#include <vector>

namespace globe {

class NoiseField {
 public:
    NoiseField(Interval output_range = Interval(0, 1));

    void normalize(const std::vector<Point3> &sample_points, Interval output_interval = Interval(0, 1));
    double value(const Point3 &location);
    Interval output_range() const { return _output_interval; }

 private:
    std::unique_ptr<anl::CKernel> _kernel;
    std::unique_ptr<anl::CInstructionIndex> _instruction_index;

    static anl::CInstructionIndex initialize_kernel(anl::CKernel &kernel);
    double noise_value(const Point3 &location);

    Interval _noise_interval;
    Interval _output_interval;
};

inline NoiseField::NoiseField(Interval output_range) :
    _kernel(std::make_unique<anl::CKernel>()),
    _instruction_index(std::make_unique<anl::CInstructionIndex>(initialize_kernel(*_kernel))),
    _noise_interval(Interval(0.0, 1.0)),
    _output_interval(output_range) {
}

inline void NoiseField::normalize(const std::vector<Point3> &sample_points, Interval output_interval) {
    auto noise_samples = sample_points | std::views::transform(
        [this](const Point3 &point) {
            return noise_value(point);
        }
    );

    _noise_interval = Interval(noise_samples);
    _output_interval = output_interval;
}

inline double NoiseField::value(const Point3 &location) {
    return Interval::map(_noise_interval, _output_interval, noise_value(location));
}

inline double NoiseField::noise_value(const Point3 &location) {
    return anl::CNoiseExecutor(*_kernel).evaluateScalar(
        location.x(), location.y(), location.z(), *_instruction_index
    );
}

inline anl::CInstructionIndex NoiseField::initialize_kernel(anl::CKernel &kernel) {
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
    instruction_index = kernel.clamp(
        instruction_index,
        kernel.constant(0.0),
        kernel.constant(1.0)
    );

    return instruction_index;
}

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_NOISE_GENERATOR_NOISE_FIELD_H_

