#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_INTEGRATION_RESULT_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_INTEGRATION_RESULT_HPP_

#include <cstddef>

namespace globe {

struct IntegrationResult {
    double mass;
    double variance;
    size_t sample_count;
};

}

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_INTEGRATION_RESULT_HPP_
