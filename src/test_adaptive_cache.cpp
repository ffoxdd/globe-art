#include "globe/integrable_field/adaptive_cache_integrable_field.hpp"
#include "globe/scalar_field/noise_field.hpp"
#include <iostream>

using namespace globe;

int main() {
    std::cout << "Initializing adaptive cache integrable field with noise field..." << std::endl;

    NoiseField noise_field;
    AdaptiveCacheIntegrableField<NoiseField&> integrable_field(noise_field, 8);

    std::cout << "Adaptive cache initialization complete!" << std::endl;

    return 0;
}
