#ifndef GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_MONTE_CARLO_INTEGRABLE_FIELD_H_
#define GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_MONTE_CARLO_INTEGRABLE_FIELD_H_

#include "../scalar_field/scalar_field.hpp"
#include "../globe_generator/spherical_polygon.hpp"
#include "../globe_generator/spherical_bounding_box.hpp"
#include "../globe_generator/monte_carlo_integrator.hpp"
#include "../globe_generator/monte_carlo_params.hpp"
#include "../globe_generator/sample_point_generator/bounding_box_sample_point_generator.hpp"
#include "../scalar_field/interval.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <array>
#include <iostream>

namespace globe {

static constexpr std::array<MonteCarloParams, 8> MONTE_CARLO_CANDIDATES = {{
    {3, 100},
    {3, 150},
    {5, 250},
    {8, 500},
    {10, 1000},
    {15, 2000},
    {20, 3500},
    {25, 5000},
}};

template<ScalarField SF>
class MonteCarloIntegrableField {
 public:
    explicit MonteCarloIntegrableField(SF scalar_field);

    double integrate(const SphericalPolygon &polygon);
    double integrate_entire_sphere();

    void calibrate(
        const std::vector<SphericalPolygon> &polygons,
        double max_coefficient_of_variation = 0.008,
        size_t num_trials = 5
    );

private:
    SF _scalar_field;
    MonteCarloParams _params;

    struct SampleOutcome {
        double mass;
        size_t sample_count;
    };

    struct CandidateMetrics {
        double mean = 0.0;
        double stddev = 0.0;
        double coefficient_of_variation = 0.0;
        double avg_sample_count = 0.0;
    };

    CandidateMetrics compute_stats(
        const SphericalPolygon &polygon,
        const MonteCarloParams &params,
        size_t num_trials
    );

    template<typename Collection, typename ValueExtractor>
    static double compute_standard_deviation(
        const Collection &values,
        double mean,
        ValueExtractor extractor
    );

    template<typename Collection>
    static double compute_standard_deviation(const Collection &values, double mean);
};

template<ScalarField SF>
MonteCarloIntegrableField<SF>::MonteCarloIntegrableField(
    SF scalar_field
) :
    _scalar_field(scalar_field),
    _params(MonteCarloParams::balanced()) {
}

template<ScalarField SF>
double MonteCarloIntegrableField<SF>::integrate(const SphericalPolygon &polygon) {
    SphericalBoundingBox bbox = polygon.bounding_box();

    MonteCarloIntegrator<SF, BoundingBoxSamplePointGenerator> calculator(
        std::ref(polygon),
        _scalar_field,
        BoundingBoxSamplePointGenerator(),
        _params,
        bbox
    );

    return calculator.integrate();
}

template<ScalarField SF>
double MonteCarloIntegrableField<SF>::integrate_entire_sphere() {
    SphericalBoundingBox bbox(
        Interval(0, 2 * M_PI),
        Interval(-1, 1)
    );

    MonteCarloIntegrator<SF, BoundingBoxSamplePointGenerator> calculator(
        std::nullopt,
        _scalar_field,
        BoundingBoxSamplePointGenerator(),
        _params,
        bbox
    );

    return calculator.integrate();
}

template<ScalarField SF>
typename MonteCarloIntegrableField<SF>::CandidateMetrics MonteCarloIntegrableField<SF>::compute_stats(
    const SphericalPolygon &polygon,
    const MonteCarloParams &params,
    size_t num_trials
) {
    std::vector<SampleOutcome> samples;
    samples.reserve(num_trials);

    for (size_t i = 0; i < num_trials; i++) {
        SphericalBoundingBox bbox = polygon.bounding_box();

        MonteCarloIntegrator<SF, BoundingBoxSamplePointGenerator> calculator(
            std::ref(polygon),
            _scalar_field,
            BoundingBoxSamplePointGenerator(),
            params,
            bbox
        );

        double mass = calculator.integrate();
        samples.push_back(SampleOutcome{mass, 0});
    }

    CandidateMetrics stats{};

    for (const auto &sample : samples) {
        stats.mean += sample.mass;
    }
    stats.mean /= num_trials;

    stats.stddev = compute_standard_deviation(
        samples,
        stats.mean,
        [](const SampleOutcome &sample) {
            return sample.mass;
        }
    );

    stats.coefficient_of_variation = (stats.mean > 0) ? (stats.stddev / stats.mean) : 0.0;
    stats.avg_sample_count = 0.0;

    return stats;
}

template<ScalarField SF>
template<typename Collection, typename ValueExtractor>
double MonteCarloIntegrableField<SF>::compute_standard_deviation(
    const Collection &values,
    double mean,
    ValueExtractor extractor
) {
    if (values.empty()) {
        return 0.0;
    }

    double variance = 0.0;
    for (const auto &value : values) {
        double diff = extractor(value) - mean;
        variance += diff * diff;
    }

    return std::sqrt(variance / static_cast<double>(values.size()));
}

template<ScalarField SF>
template<typename Collection>
double MonteCarloIntegrableField<SF>::compute_standard_deviation(
    const Collection &values,
    double mean
) {
    return compute_standard_deviation(
        values,
        mean,
        [](const auto &value) {
            return value;
        }
    );
}

template<ScalarField SF>
void MonteCarloIntegrableField<SF>::calibrate(
    const std::vector<SphericalPolygon> &polygons,
    double max_coefficient_of_variation,
    size_t num_trials
) {
    MonteCarloParams best_params{};
    CandidateMetrics best_metrics{
        .coefficient_of_variation = std::numeric_limits<double>::infinity()
    };

    size_t candidate_index = 0;
    for (const auto &params : MONTE_CARLO_CANDIDATES) {
        candidate_index++;
        std::cout <<
            "  Candidate " << candidate_index << "/" << MONTE_CARLO_CANDIDATES.size() <<
            ": stable iterations = " << params.consecutive_stable_iterations <<
            ", min hits = " << params.min_hits << std::endl;

        CandidateMetrics observation{};

        for (const auto &polygon : polygons) {
            auto stats = compute_stats(polygon, params, num_trials);
            observation.stddev += stats.stddev;
            observation.avg_sample_count += stats.avg_sample_count;
            observation.coefficient_of_variation = std::max(
                observation.coefficient_of_variation,
                stats.coefficient_of_variation
            );
        }

        observation.stddev /= polygons.size();
        observation.avg_sample_count /= polygons.size();

        std::cout <<
            "    Avg samples per cell: " << observation.avg_sample_count <<
            ", avg stddev: " << observation.stddev <<
            ", max coefficient of variation: " << observation.coefficient_of_variation <<
            std::endl;

        if (observation.coefficient_of_variation <= max_coefficient_of_variation) {
            _params = params;

            std::cout <<
                "Calibration complete:\n" <<
                "  Stable iterations: " << params.consecutive_stable_iterations << "\n" <<
                "  Min hits: " << params.min_hits << "\n" <<
                "  Avg samples per cell: " << observation.avg_sample_count << "\n" <<
                "  Avg stddev: " << observation.stddev << "\n" <<
                "  Max coefficient of variation: " << observation.coefficient_of_variation <<
                " (target: " << max_coefficient_of_variation << ")\n\n";

            return;
        }

        if (observation.coefficient_of_variation < best_metrics.coefficient_of_variation) {
            best_params = params;
            best_metrics.stddev = observation.stddev;
            best_metrics.avg_sample_count = observation.avg_sample_count;
            best_metrics.coefficient_of_variation = observation.coefficient_of_variation;
        }
    }

    _params = best_params;
    std::cout <<
        "Calibration complete (fallback):\n" <<
        "  Stable iterations: " << best_params.consecutive_stable_iterations << "\n" <<
        "  Min hits: " << best_params.min_hits << "\n" <<
        "  Avg samples per cell: " << best_metrics.avg_sample_count << "\n" <<
        "  Avg stddev: " << best_metrics.stddev << "\n" <<
        "  Max coefficient of variation: " << best_metrics.coefficient_of_variation <<
        " (target: " << max_coefficient_of_variation << ")\n\n";
}

}

#endif //GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_MONTE_CARLO_INTEGRABLE_FIELD_H_
