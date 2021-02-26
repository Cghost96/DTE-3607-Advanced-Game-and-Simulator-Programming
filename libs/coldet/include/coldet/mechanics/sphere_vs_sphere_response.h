#ifndef DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H
#define DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H

#include "../bits/types.h"
#include "../bits/rigidbodies.h"
#include "../mechanics/compute_trajectory.h"
#include "../utils/state_computations.h"
#include "../utils/energy_computations.h"

#include <blaze/Math.h>
#include <cmath>

namespace dte3607::coldet::mechanics {

  using States          = rigidbodies::Sphere::States;
  using Sphere          = rigidbodies::Sphere;
  using FixedSphere     = rigidbodies::FixedSphere;
  using FixedPlane      = rigidbodies::FixedPlane;
  using OsFpAttachments = std::unordered_map<Sphere*, FixedPlane*>;
  using V3              = types::Vector3;
  using P3              = types::Point3;
  using VT              = types::ValueType;

  inline std::pair<V3, V3> computeImpactResponseSphereSphere(P3 const& p1, V3 const& v1, VT const& m1,
                                                             P3 const& p2, V3 const& v2, VT const& m2) {
    auto const d          = blaze::evaluate(blaze::normalize(p2 - p1));
    auto const inner_v1_d = blaze::inner(v1, d) * d;
    auto const inner_v2_d = blaze::inner(v2, d) * d;
    auto const v1_remain  = v1 - inner_v1_d;
    auto const v2_remain  = v2 - inner_v2_d;
    auto const v_prime1_d = (((m1 - m2) / (m1 + m2)) * inner_v1_d) + (((2 * m2) / (m1 + m2)) * inner_v2_d);
    auto const v_prime2_d = (((m2 - m1) / (m1 + m2)) * inner_v2_d) + (((2 * m1) / (m1 + m2)) * inner_v1_d);
    auto const v_prime1   = v1_remain + v_prime1_d;
    auto const v_prime2   = v2_remain + v_prime2_d;

    return {v_prime1, v_prime2};
  }

  inline std::tuple<V3, V3, V3, V3>
  computeImpactResponseSphereSphereWithStates(Sphere* s1, Sphere* s2,
                                              OsFpAttachments const& attachmentsOsFp) {
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

    auto [v1_original, v2_original] = computeImpactResponseSphereSphere(
      s1->point(), s1->velocity(), s1->mass(), s2->point(), s2->velocity(), s2->mass());

    auto getAdjustedVelocity = [&attachmentsOsFp, &getParallelVelocity](Sphere* o, V3 v) -> V3 {
      if (o->state() == States::Free) return v;
      auto const attachedPlane = attachmentsOsFp.at(o);
      auto       v_adjusted    = getParallelVelocity(v, attachedPlane->normal());
      return v_adjusted;
    };

    return std::make_tuple(getAdjustedVelocity(s1, v1_original), v1_original,
                           getAdjustedVelocity(s2, v2_original), v2_original);
  }

  inline std::tuple<V3, V3, V3, V3>
  computeImpactResponseSphereSphereWithStates(Sphere* s1, Sphere* s2, OsFpAttachments const& attachmentsOsFp,
                                              VT const& µ1, VT const& µ2) {
    auto [v1_adj, v1_orig, v2_adj, v2_orig]
      = computeImpactResponseSphereSphereWithStates(s1, s2, attachmentsOsFp);
    utils::energy::dampening::addTimeIndependentLoss(v1_adj, µ1, µ2);
    utils::energy::dampening::addTimeIndependentLoss(v1_orig, µ1, µ2);
    utils::energy::dampening::addTimeIndependentLoss(v2_adj, µ1, µ2);
    utils::energy::dampening::addTimeIndependentLoss(v2_orig, µ1, µ2);
    return std::make_tuple(v1_adj, v1_orig, v2_adj, v2_orig);
  }

  inline V3 computeImpactResponseSphereFixedSphere(Sphere* s, FixedSphere* fs) {
    // Computation similar to plane-response
    V3 const   n          = s->point() - fs->point();
    V3 const   n_norm     = blaze::normalize(n);
    V3 const   v          = s->velocity();
    auto const inner_v_nn = blaze::inner(v, n_norm);
    V3         response   = v - 2 * inner_v_nn * n_norm;
    utils::energy::dampening::addTimeIndependentLoss(response, s->frictionCoef(), fs->frictionCoef());

    return response;
  }
}   // namespace dte3607::coldet::mechanics


#endif   // DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H
