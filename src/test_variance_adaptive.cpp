#include "globe/integrable_field/variance_adaptive_integrable_field.hpp"
#include "globe/scalar_field/noise_field.hpp"
#include <iostream>

using namespace globe;

int main() {
    std::cout << "Initializing variance-adaptive integrable field with noise field..." << std::endl;

    NoiseField noise_field;
    VarianceAdaptiveIntegrableField<NoiseField&> integrable_field(
        noise_field,
        0.01,   // variance_threshold
        0.001,  // min_mass_threshold
        12      // max_depth
    );

    std::cout << "\nVariance-adaptive field initialization complete!" << std::endl;

    return 0;
}
