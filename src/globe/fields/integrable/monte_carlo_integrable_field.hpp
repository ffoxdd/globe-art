#ifndef GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_MONTE_CARLO_INTEGRABLE_FIELD_H_
#define GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_MONTE_CARLO_INTEGRABLE_FIELD_H_

#include "../scalar/scalar_field.hpp"
#include "../../geometry/spherical/spherical_polygon.hpp"
#include "../../geometry/spherical/spherical_bounding_box.hpp"
#include "monte_carlo_integrator.hpp"
#include "../../generators/random_sphere_point_generator.hpp"
#include <optional>

namespace globe {

template<ScalarField SF>
class MonteCarloIntegrableField {
 public:
    explicit MonteCarloIntegrableField(SF scalar_field);

    double integrate(const SphericalPolygon &polygon);
    double integrate(const SphericalBoundingBox &bbox = SphericalBoundingBox::full_sphere());

private:
    SF _scalar_field;

};

template<ScalarField SF>
MonteCarloIntegrableField<SF>::MonteCarloIntegrableField(
    SF scalar_field
) :
    _scalar_field(scalar_field) {
}

template<ScalarField SF>
double MonteCarloIntegrableField<SF>::integrate(const SphericalPolygon &polygon) {
    SphericalBoundingBox bbox = polygon.bounding_box();

    MonteCarloIntegrator<SF, RandomSpherePointGenerator> calculator(
        std::ref(polygon),
        _scalar_field,
        RandomSpherePointGenerator(),
        bbox
    );

    return calculator.integrate();
}

template<ScalarField SF>
double MonteCarloIntegrableField<SF>::integrate(const SphericalBoundingBox &bbox) {
    MonteCarloIntegrator<SF, RandomSpherePointGenerator> calculator(
        std::nullopt,
        _scalar_field,
        RandomSpherePointGenerator(),
        bbox
    );

    return calculator.integrate();
}

}

#endif //GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_MONTE_CARLO_INTEGRABLE_FIELD_H_
