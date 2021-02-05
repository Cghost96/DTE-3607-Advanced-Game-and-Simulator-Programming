#ifndef DTE3607_COLDET_MECHANICS_COMPUTE_TRAJECTORY_H
#define DTE3607_COLDET_MECHANICS_COMPUTE_TRAJECTORY_H

#include "../bits/types.h"
#include "../bits/rigidbodies.h"
#include "../utils/type_conversion.h"
#include "../utils/energy_computations.h"

namespace dte3607::coldet::mechanics {

  using States         = rigidbodies::Sphere::States;
  using Sphere         = rigidbodies::Sphere;
  using FixedPlane     = rigidbodies::FixedPlane;
  using SFPAttachments = std::unordered_map<Sphere*, FixedPlane*>;
  using P3             = types::Point3;
  using V3             = types::Vector3;
  using VT             = types::ValueType;
  using NS             = types::NanoSeconds;

  V3 getParallelDs(const V3 ds, const V3 n);
  V3 getParallelAcceleration(V3 const a, V3 const n);

  inline std::pair<V3, V3> computeLinearTrajectory(V3 const& velocity, V3 const& force, NS const timestep) {
    auto const   v  = velocity;
    auto const   F  = force;
    double const dt = utils::toDt(timestep);
    auto const   a  = F * dt;
    auto const   ds = (v + (1.0 / 2.0) * a) * dt;

    return {ds, a};
  }

  inline std::pair<V3, V3> computeRollingLinTraj(V3 const& velocity, V3 const& a, NS const timestep) {
    auto const   v  = velocity;
    double const dt = utils::toDt(timestep);
    auto const   ds = (v + (1.0 / 2.0) * a * dt) * dt;

    return std::make_pair(ds, a * dt);
  }

  inline std::pair<V3, V3> computeLinearTrajectory(Sphere* o, SFPAttachments const& attachments, V3 force,
                                                   NS dt) {
    auto const plane = attachments.at(o);
    auto const n     = plane->normal();
    auto const µ1    = o->frictionCoef();
    auto const µ2    = plane->frictionCoef();

    auto adjustedTrajectory = [&n](V3 ds, V3 a) -> std::pair<V3, V3> {
      auto const ds_adj = getParallelDs(ds, n);
      auto const a_adj  = getParallelAcceleration(a, n);
      return std::make_pair(ds_adj, a_adj);
    };

    if (o->state() == States::Sliding) {
      force        = utils::energy::force::computeSlidingForce(force, µ1, µ2);
      auto [ds, a] = computeLinearTrajectory(o->velocity(), force, dt);
      return adjustedTrajectory(ds, a);
    }
    else if (o->state() == States::Rolling) {
      force
        = utils::energy::force::computeRollingForce(o->mass(), o->radius(), force, µ1, µ2, o->velocity(), n);
      auto a = utils::energy::motion::getTotalRollingAcceleration(force, o->mass(), µ1, µ2, o->radius(),
                                                                  o->velocity(), n);
      return computeRollingLinTraj(o->velocity(), a, dt);
    }
  }

  // Helpers
  V3 getParallelAcceleration(V3 const a, V3 const n) {
    auto const inner_n_a = blaze::evaluate(blaze::inner(blaze::normalize(n), a));

    if (inner_n_a == 0)
      return a;
    else {
      auto const n_scaled = V3{n[0] * inner_n_a, n[1] * inner_n_a, n[2] * inner_n_a};
      return V3{a[0] - n_scaled[0], a[1] - n_scaled[1], a[2] - n_scaled[2]};
    }
  }

  V3 getParallelDs(V3 const ds, V3 const n) {
    auto const ortho       = blaze::cross(ds, n);
    auto const ds_prime    = blaze::cross(n, ortho);
    auto const ds_parallel = blaze::normalize(ds_prime) * blaze::inner(ds, ds_prime);
    return ds_parallel;
  }

}   // namespace dte3607::coldet::mechanics

#endif   // DTE3607_COLDET_MECHANICS_COMPUTE_TRAJECTORY_H
