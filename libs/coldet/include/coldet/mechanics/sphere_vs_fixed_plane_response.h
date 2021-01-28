#ifndef DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H
#define DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H

#include "../bits/types.h"
#include "../bits/rigidbodies.h"
#include "../utils/energy_computations.h"

#include <blaze/Math.h>

namespace dte3607::coldet::mechanics {

  using States     = rigidbodies::Sphere::States;
  using Sphere     = rigidbodies::Sphere;
  using FixedPlane = rigidbodies::FixedPlane;
  using V3         = types::Vector3;
  using VT         = types::ValueType;

  V3 getParallelVelocity(V3 v, V3 n);

  // General
  inline V3 computeImpactResponseSphereFixedPlane(V3 const& v, V3 const& n) {
    auto const inner_v_n = blaze::inner(v, n);
    auto const response  = v - 2 * inner_v_n * n;

    return response;
  }

  // States
  inline V3 computeImpactResponseSphereFixedPlane(Sphere* o, V3 const& n) {
    auto v = computeImpactResponseSphereFixedPlane(o->velocity(), n);
    return o->state() == States::Free ? v : getParallelVelocity(v, n);
  }

  // States and friction
  inline V3 computeImpactResponseSphereFixedPlane(Sphere* o, FixedPlane* f) {
    auto v = computeImpactResponseSphereFixedPlane(o, f->normal());
    utils::energy::dampening::addTimeIndependentLoss(v, o->frictionCoef(), f->frictionCoef());
    return v;
  }

  // Helpers
  V3 getParallelVelocity(V3 v, V3 n) {
    auto const n_normalized    = blaze::normalize(n);
    auto const v_normalized    = blaze::normalize(v);
    auto const inner_n_v       = blaze::evaluate(blaze::inner(n_normalized, v));
    bool const isSameDirection = (n_normalized[0] - v_normalized[0]) == 0
                                 && (n_normalized[1] - v_normalized[1]) == 0
                                 && (n_normalized[2] - v_normalized[2]) == 0;

    if (inner_n_v == 0 || isSameDirection)
      return v;
    else {
      auto const n_scaled = V3{n[0] * inner_n_v, n[1] * inner_n_v, n[2] * inner_n_v};
      return V3{v[0] - n_scaled[0], v[1] - n_scaled[1], v[2] - n_scaled[2]};
    }
  }

}   // namespace dte3607::coldet::mechanics


#endif   // DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H
