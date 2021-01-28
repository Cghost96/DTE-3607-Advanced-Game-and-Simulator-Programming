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

  using States         = rigidbodies::Sphere::States;
  using Sphere         = rigidbodies::Sphere;
  using FixedPlane     = rigidbodies::FixedPlane;
  using SFPAttachments = std::unordered_map<Sphere*, FixedPlane*>;
  using V3             = types::Vector3;
  using P3             = types::Point3;
  using VT             = types::ValueType;

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
  computeImpactResponseSphereSphereWithStates(Sphere* o1, Sphere* o2, SFPAttachments const& attachments) {
    auto [v1_original, v2_original] = computeImpactResponseSphereSphere(
      o1->point(), o1->velocity(), o1->mass(), o2->point(), o2->velocity(), o2->mass());

    auto getAdjustedVelocity = [&attachments](Sphere* o, V3 v) -> V3 {
      if (o->state() == States::Free) return v;
      auto const attachedPlane = attachments.at(o);
      auto       v_adjusted    = getParallelVelocity(v, attachedPlane->normal());   // Helper in S-P response
      return v_adjusted;
    };

    return std::make_tuple(getAdjustedVelocity(o1, v1_original), v1_original,
                           getAdjustedVelocity(o2, v2_original), v2_original);
  }

  inline std::tuple<V3, V3, V3, V3>
  computeImpactResponseSphereSphereWithStates(Sphere* o1, Sphere* o2, SFPAttachments const& attachments,
                                              VT const& µ1, VT const& µ2) {
    auto [v1_adj, v1_orig, v2_adj, v2_orig]
      = computeImpactResponseSphereSphereWithStates(o1, o2, attachments);
    utils::energy::dampening::addTimeIndependentLoss(v1_adj, µ1, µ2);
    utils::energy::dampening::addTimeIndependentLoss(v1_orig, µ1, µ2);
    utils::energy::dampening::addTimeIndependentLoss(v2_adj, µ1, µ2);
    utils::energy::dampening::addTimeIndependentLoss(v2_orig, µ1, µ2);
    return std::make_tuple(v1_adj, v1_orig, v2_adj, v2_orig);
  }

}   // namespace dte3607::coldet::mechanics


#endif   // DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H
