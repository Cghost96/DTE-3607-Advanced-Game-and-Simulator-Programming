#ifndef DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H
#define DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H

#include "../bits/types.h"
#include "../bits/rigidbodies.h"
#include "../utils/energy_computations.h"

#include <blaze/Math.h>

namespace dte3607::coldet::mechanics {

  using States            = rigidbodies::Sphere::States;
  using Sphere            = rigidbodies::Sphere;
  using FixedPlane        = rigidbodies::FixedPlane;
  using FixedLimitedPlane = rigidbodies::FixedLimitedPlane;
  using V3                = types::Vector3;
  using VT                = types::ValueType;

  V3 getParallelVelocity(V3 v, V3 n);

  // General
  inline V3 computeImpactResponseSphereFixedPlane(V3 const& v, V3 const& n) {
    auto const n_norm    = blaze::normalize(n);
    auto const inner_v_n = blaze::inner(v, n_norm);
    auto const response  = v - 2 * inner_v_n * n_norm;

    return response;
  }

  // States
  inline V3 computeImpactResponseSphereFixedPlane(Sphere* s, V3 const& n) {
    auto getParallelVelocity = [](V3 v, V3 n) -> V3 {
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
    };

    auto const n_norm = blaze::normalize(n);
    auto       v      = computeImpactResponseSphereFixedPlane(s->velocity(), n_norm);
    return s->state() == States::Free ? v : getParallelVelocity(v, n_norm);
  }

  // States and friction
  inline V3 computeImpactResponseSphereFixedPlane(Sphere* s, FixedPlane* p) {
    auto v = computeImpactResponseSphereFixedPlane(s, p->normal());
    utils::energy::dampening::addTimeIndependentLoss(v, s->frictionCoef(), p->frictionCoef());
    return v;
  }

  inline V3 computeImpactResponseSphereFixedLimitedPlane(Sphere* s, FixedLimitedPlane* lp) {
    auto const n = blaze::evaluate(blaze::normalize(lp->normalFront()));
    auto       v = computeImpactResponseSphereFixedPlane(s, n);
    utils::energy::dampening::addTimeIndependentLoss(v, s->frictionCoef(), lp->frictionCoef());
    return v;
  }
}   // namespace dte3607::coldet::mechanics


#endif   // DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H
