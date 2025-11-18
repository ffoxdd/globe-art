#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_MONTE_CARLO_RESULT_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_MONTE_CARLO_RESULT_HPP_

namespace globe {

struct MonteCarloResult {
    double mass;
    double variance;
    size_t sample_count;
};

}

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_MONTE_CARLO_RESULT_HPP_
