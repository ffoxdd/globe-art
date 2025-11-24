#ifndef GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_MONTE_CARLO_INTEGRABLE_FIELD_H_
#define GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_MONTE_CARLO_INTEGRABLE_FIELD_H_

#include "../scalar/scalar_field.hpp"
#include "../../geometry/spherical/spherical_polygon.hpp"
#include "../../geometry/spherical/spherical_bounding_box.hpp"
#include "monte_carlo_integrator.hpp"
#include "../../generators/sphere_point_generator.hpp"
#include <optional>

namespace globe {

template<ScalarField SF, SpherePointGenerator GeneratorType>
class MonteCarloIntegrableField {
 public:
    MonteCarloIntegrableField(SF scalar_field, GeneratorType point_generator);

    double integrate(const SphericalPolygon &polygon);
    double integrate(const SphericalBoundingBox &bbox = SphericalBoundingBox::full_sphere());

private:
    SF _scalar_field;
    GeneratorType _point_generator;

};

template<ScalarField SF, SpherePointGenerator GeneratorType>
MonteCarloIntegrableField<SF, GeneratorType>::MonteCarloIntegrableField(
    SF scalar_field,
    GeneratorType point_generator
) :
    _scalar_field(scalar_field),
    _point_generator(std::move(point_generator)) {
}

template<ScalarField SF, SpherePointGenerator GeneratorType>
double MonteCarloIntegrableField<SF, GeneratorType>::integrate(const SphericalPolygon &polygon) {
    MonteCarloIntegrator<SF, GeneratorType> calculator(
        polygon,
        _scalar_field,
        _point_generator
    );

    return calculator.integrate();
}

template<ScalarField SF, SpherePointGenerator GeneratorType>
double MonteCarloIntegrableField<SF, GeneratorType>::integrate(const SphericalBoundingBox &bbox) {
    MonteCarloIntegrator<SF, GeneratorType> calculator(
        _scalar_field,
        _point_generator
    );

    return calculator.integrate();
}

}

#endif //GLOBEART_SRC_GLOBE_INTEGRABLE_FIELD_MONTE_CARLO_INTEGRABLE_FIELD_H_
