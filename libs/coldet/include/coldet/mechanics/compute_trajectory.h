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

  P3 getNearestPointOnPlane(const P3 q_original, const P3 p, const V3 ds, const V3 n);
  V3 getParallelDs(const V3 ds, const V3 n, const P3 p, const VT r, const P3 q_nearest);
  V3 getParallelAcceleration(V3 const a, V3 const n);
  // TODO add prototype for contact surface normal

  inline std::pair<V3, V3> computeLinearTrajectory(V3 const& velocity, V3 const& force, NS const timestep) {
    auto const   v  = velocity;
    auto const   F  = force;
    double const dt = utils::toDt(timestep);
    auto const   a  = F * dt;
    auto const   ds = (v + (1.0 / 2.0) * a) * dt;

    return {ds, a};
  }

  inline std::pair<V3, V3> computeLinearTrajectory(Sphere* o, SFPAttachments const& attachments, V3 force,
                                                   NS const timestep) {

    if (o->state() == States::Sliding) {
      auto const µ1 = o->frictionCoef();
      auto const µ2 = attachments.at(o)->frictionCoef();
      force         = utils::energy::force::computeSlidingForce(force, µ1, µ2);
    }
    auto [ds, a] = computeLinearTrajectory(o->velocity(), force, timestep);

    if (o->state() == States::Free)
      return {ds, a};
    else {
      auto const attachedPlane = attachments.at(o);
      auto const n             = attachedPlane->normal();
      auto const q             = attachedPlane->point();
      auto const p             = o->point();
      auto const r             = o->radius();
      auto const q_nearest     = getNearestPointOnPlane(q, p, ds, n);
      auto const ds_parallel   = getParallelDs(ds, n, p, r, q_nearest);
      auto const a_parallel    = getParallelAcceleration(a, n);

      return {ds_parallel, a_parallel};
    }
  }

  // Helpers
  P3 getNearestPointOnPlane(P3 const q_original, P3 const p, V3 const ds, V3 const n) {
    // https://slideplayer.com/slide/4550995/ ----> Q = p_prime, R = q, n = n, P = p
    const P3 p_prime = P3{p[0] + ds[0], p[1] + ds[1], p[2] + ds[2]};
    const V3 qp_prime
      = V3{p_prime[0] - q_original[0], p_prime[1] - q_original[1], p_prime[2] - q_original[2]};
    const double n_qp_prime = (n[0] * qp_prime[0]) + (n[1] * qp_prime[1]) + (n[2] * qp_prime[2]);
    const double nn         = (n[0] * n[0]) + (n[1] * n[1]) + (n[2] * n[2]);
    const double t          = n_qp_prime / nn;
    const V3     tn         = V3{t * n[0], t * n[1], t * n[2]};
    const P3     q          = P3{p_prime[0] - tn[0], p_prime[1] - tn[1], p_prime[2] - tn[2]};

    return q;
  }

  V3 getParallelAcceleration(V3 const a, V3 const n) {
    auto const inner_n_a = blaze::evaluate(blaze::inner(blaze::normalize(n), a));

    if (inner_n_a == 0)
      return a;
    else {
      auto const n_scaled = V3{n[0] * inner_n_a, n[1] * inner_n_a, n[2] * inner_n_a};
      return V3{a[0] - n_scaled[0], a[1] - n_scaled[1], a[2] - n_scaled[2]};
    }
  }

  V3 getParallelDs(V3 const ds, V3 const n, P3 const p, VT const r, P3 const q_nearest) {
    auto const p_prime     = p + ds;
    auto const d           = (q_nearest - p_prime) + (blaze::normalize(n) * r);
    auto const ds_adjusted = ds + d;

    return ds_adjusted;
  }

  /*V3 getContactSurfaceNormal(const P3 &sphere_p, const AttachedPlanes &F,
  const AttachedSpheres &O); inline V3 getContactSurfaceNormal(const P3
  &sphere_p, const AttachedPlanes &F, const AttachedSpheres &O) { V3 n = V3{0,
  0, 0};

      if(not F.empty()) {
          for(auto& f : F) {
              n[0] += f->normal()[0];
              n[1] += f->normal()[1];
              n[2] += f->normal()[2];
          }
      }

      if(not O.empty()) {
          for(auto& o : O) {
              VT const px = sphere_p[0], py = sphere_p[1], pz = sphere_p[2];
              VT const ox = o->point()[0], oy = o->point()[1], oz =
  o->point()[2]; V3 const po = blaze::normalize(V3{ox - px, oy - py, oz - pz});
              //// set length of po = radius of sphere
  //            VT const factor = r / blaze::length(po);
  //            V3 const po_adjusted = V3{po[0] * factor, po[1] * factor, po[2]
  * factor};

              // Normal of the imaginary tangential plane between the two
  spheres (pointing into the sphere)
  //            V3 const sphere_n = po_adjusted * -1;
              V3 const sphere_n = po * -1;
              // Add to n
              n[0] += sphere_n[0];
              n[1] += sphere_n[1];
              n[2] += sphere_n[2];
          }
      }

      return blaze::normalize(n);
  }*/

}   // namespace dte3607::coldet::mechanics

#endif   // DTE3607_COLDET_MECHANICS_COMPUTE_TRAJECTORY_H
