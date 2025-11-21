#include <anl/anl.h>
#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>

anl::CInstructionIndex initialize_kernel(anl::CKernel &kernel) {
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

int main() {
    anl::CKernel kernel;
    auto instruction_index = initialize_kernel(kernel);
    anl::CNoiseExecutor executor(kernel);

    double min_value = std::numeric_limits<double>::max();
    double max_value = std::numeric_limits<double>::lowest();

    const int sample_count = 100000;
    std::vector<double> samples;
    samples.reserve(sample_count);

    std::cout << "Sampling " << sample_count << " points from ANL noise..." << std::endl;

    for (int i = 0; i < sample_count; ++i) {
        double x = (double)rand() / RAND_MAX * 10.0 - 5.0;
        double y = (double)rand() / RAND_MAX * 10.0 - 5.0;
        double z = (double)rand() / RAND_MAX * 10.0 - 5.0;

        double value = executor.evaluateScalar(x, y, z, instruction_index);
        samples.push_back(value);

        min_value = std::min(min_value, value);
        max_value = std::max(max_value, value);
    }

    std::cout << std::fixed << std::setprecision(10);
    std::cout << "\nRaw ANL output range:" << std::endl;
    std::cout << "  Min: " << min_value << std::endl;
    std::cout << "  Max: " << max_value << std::endl;
    std::cout << "  Range: " << (max_value - min_value) << std::endl;

    double sum = 0.0;
    for (double v : samples) {
        sum += v;
    }
    double mean = sum / sample_count;

    double variance_sum = 0.0;
    for (double v : samples) {
        variance_sum += (v - mean) * (v - mean);
    }
    double std_dev = std::sqrt(variance_sum / sample_count);

    std::cout << "\nStatistics:" << std::endl;
    std::cout << "  Mean: " << mean << std::endl;
    std::cout << "  Std Dev: " << std_dev << std::endl;

    std::cout << "\nTo scale to [0, 1], use:" << std::endl;
    std::cout << "  scaled = (value - " << min_value << ") / " << (max_value - min_value) << std::endl;

    return 0;
}
