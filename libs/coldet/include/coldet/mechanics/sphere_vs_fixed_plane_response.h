#ifndef DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H
#define DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H

#include "../bits/types.h"

#include <blaze/Math.h>

namespace dte3607::coldet::mechanics {


inline types::Vector3 computeImpactResponseSphereFixedPlane(
    [[maybe_unused]] types::Vector3 const& sphere_v,
    [[maybe_unused]] types::Vector3 const& fplane_n) {

    auto const v = sphere_v;
    auto const n = fplane_n;

    auto const inner_v_n = blaze::inner(v, n);
    auto const v_prime = v - 2 * inner_v_n * n; // v'

    return {v_prime};
}


}   // namespace dte3607::coldet::mechanics


#endif // DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H
