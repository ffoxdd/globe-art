#ifndef GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_MONTE_CARLO_INTEGRABLE_FIELD_H_
#define GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_MONTE_CARLO_INTEGRABLE_FIELD_H_

#include "integrable_field.hpp"
#include "../scalar_field/scalar_field.hpp"
#include "../globe_generator/spherical_polygon.hpp"
#include "../globe_generator/spherical_bounding_box.hpp"
#include "../globe_generator/monte_carlo_integrator.hpp"
#include "../globe_generator/sample_point_generator/bounding_box_sample_point_generator.hpp"
#include "../scalar_field/interval.hpp"
#include "../types.hpp"

namespace globe {

template<ScalarField SF>
class MonteCarloIntegrableField {
 public:
    explicit MonteCarloIntegrableField(SF scalar_field);
    double integrate(const SphericalPolygon &polygon);

 private:
    SF _scalar_field;
};

template<ScalarField SF>
MonteCarloIntegrableField<SF>::MonteCarloIntegrableField(SF scalar_field) :
    _scalar_field(scalar_field) {
}

template<ScalarField SF>
double MonteCarloIntegrableField<SF>::integrate(const SphericalPolygon &polygon) {
    SphericalBoundingBox bbox = polygon.bounding_box();

    MonteCarloIntegrator<SF, BoundingBoxSamplePointGenerator> calculator(
        std::ref(polygon),
        _scalar_field,
        BoundingBoxSamplePointGenerator(bbox),
        1e-6,
        10,
        bbox
    );

    return calculator.result().mass;
}

}

#endif //GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_MONTE_CARLO_INTEGRABLE_FIELD_H_
