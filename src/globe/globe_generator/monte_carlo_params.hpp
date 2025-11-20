#ifndef GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_MONTE_CARLO_PARAMS_HPP_
#define GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_MONTE_CARLO_PARAMS_HPP_

#include <cstddef>

namespace globe {

struct MonteCarloParams {
    int consecutive_stable_iterations;
    size_t min_hits;

    static MonteCarloParams conservative() {
        return MonteCarloParams{10, 1000};
    }

    static MonteCarloParams fast() {
        return MonteCarloParams{3, 100};
    }

    static MonteCarloParams balanced() {
        return MonteCarloParams{5, 250};
    }
};

}

#endif //GLOBEART_SRC_GLOBE_GLOBE_GENERATOR_MONTE_CARLO_PARAMS_HPP_
