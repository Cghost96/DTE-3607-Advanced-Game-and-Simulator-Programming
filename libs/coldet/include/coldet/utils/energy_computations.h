#ifndef ENERGY_COMPUTATIONS_H
#define ENERGY_COMPUTATIONS_H

#include "../bits/types.h"
#include "../bits/rigidbodies.h"
#include "../utils/type_conversion.h"

#include <blaze/Math.h>
#include <unordered_map>

namespace dte3607::coldet::utils::energy {

  using VT                   = types::ValueType;
  using V3                   = types::Vector3;
  using NS                   = types::NanoSeconds;
  using Sphere               = rigidbodies::Sphere;
  using FixedPlane           = rigidbodies::FixedPlane;
  using OsFpAttachments      = std::unordered_map<Sphere*, FixedPlane*>;
  using States               = rigidbodies::Sphere::States;
  auto const compressionTime = 1.;

  // Prototypes
  VT getTotalFriction(const VT µ1, const VT µ2);
  namespace motion {
    V3 getPreTangentialAcceleration(V3 const& force, VT const& m, VT const& µ1, VT const& µ2, VT const& r,
                                    V3 const& v, V3 const& n);
    VT getInertia(VT const& m, VT const& r);
  }   // namespace motion

  namespace dampening {
    inline void addTimeDependentLoss(Sphere* s, OsFpAttachments const& attachments, NS const dt) {
      if (s->state() == States::Sliding || s->state() == States::Rolling) {
        auto const µ1          = s->frictionCoef();
        auto const µ2          = attachments.at(s)->frictionCoef();
        auto const dt_seconds  = utils::toDt(dt);
        VT const   loss_factor = (getTotalFriction(µ1, µ2) * dt_seconds) / 1.;
        V3 const   new_v       = {s->velocity()[0] - (s->velocity()[0] * loss_factor),
                          s->velocity()[1] - (s->velocity()[1] * loss_factor),
                          s->velocity()[2] - (s->velocity()[2] * loss_factor)};

        s->setVelocity(new_v);
      }
    }

    inline void addTimeIndependentLoss(V3& v_response, VT const µ1, VT const µ2) {
      auto const loss_factor = std::clamp(getTotalFriction(µ1, µ2) * compressionTime, 0.0, 1.0);
      v_response
        = V3{v_response[0] - (v_response[0] * loss_factor), v_response[1] - (v_response[1] * loss_factor),
             v_response[2] - (v_response[2] * loss_factor)};
    }
  }   // namespace dampening

  namespace force {
    inline V3 computeSlidingForce(V3 const F, VT const& µ1, VT const& µ2) {
      return F - (F * getTotalFriction(µ1, µ2));
    }

    inline V3 computeRollingForce(VT const& m, VT const& r, V3 const& force, VT const& µ1, VT const& µ2,
                                  V3 const& v, V3 const& n) {
      VT const I            = motion::getInertia(m, r);
      VT const r_sq         = blaze::pow(r, 2);
      V3 const a_hat_t      = motion::getPreTangentialAcceleration(force, m, µ1, µ2, r, v, n);
      VT const R_scalar     = ((m * I) / (m * r_sq + I)) * blaze::length(a_hat_t);
      V3 const a_hat_t_norm = blaze::normalize(a_hat_t);
      V3 const R_force
        = {a_hat_t_norm[0] * -R_scalar, a_hat_t_norm[1] * -R_scalar, a_hat_t_norm[2] * -R_scalar};
      return R_force;
    }
  }   // namespace force

  namespace motion {
    inline V3 getPreTangentialAcceleration(V3 const& force, VT const& m, VT const& µ1, VT const& µ2,
                                           VT const& r, V3 const& v, V3 const& n) {
      V3 const g              = {force[0] / m, force[1] / m, force[2] / m};
      V3 const n_norm         = blaze::normalize(n);
      VT const K_n_scalar     = blaze::inner(force, n_norm);
      V3 const K_n            = {n_norm[0] * K_n_scalar, n_norm[1] * K_n_scalar, n_norm[2] * K_n_scalar};
      VT const xi             = (µ1 + µ2) * blaze::pow(r, 2) * (blaze::length(K_n) / m);
      V3 const v_norm         = blaze::normalize(v);
      VT const a_hat_inner    = blaze::inner(g, blaze::normalize(n));
      V3 const a_hat_normal   = {n_norm[0] * a_hat_inner, n_norm[1] * a_hat_inner, n_norm[2] * a_hat_inner};
      V3 const a_hat_velocity = {v_norm[0] * xi, v_norm[1] * xi, v_norm[2] * xi};
      V3 const a_hat_t        = blaze::evaluate(g - a_hat_normal - a_hat_velocity);
      return a_hat_t;
    }

    inline V3 getNormalAcceleration(V3 const& force, VT const& m, V3 const& n) {
      V3 const g      = {force[0] / m, force[1] / m, force[2] / m};
      V3 const n_norm = blaze::normalize(n);
      VT const eta    = blaze::inner(g, n_norm);
      V3 const a_n    = {n_norm[0] * eta, n_norm[1] * eta, n_norm[2] * eta};
      return a_n;
    }

    inline VT getInertia(VT const& m, VT const& r) { return (2. / 5.) * m * blaze::pow(r, 2); }

    inline V3 getTangentialAcceleration(V3 const& force, VT const& m, VT const& µ1, VT const& µ2, VT const& r,
                                        V3 const& v, V3 const& n) {
      V3 const a_hat_t  = getPreTangentialAcceleration(force, m, µ1, µ2, r, v, n);
      V3 const R        = force::computeRollingForce(m, r, force, µ1, µ2, v, n);
      V3 const R_over_m = {R[0] / m, R[1] / m, R[2] / m};
      V3 const a_t      = blaze::evaluate(a_hat_t + R_over_m);
      return a_t;
    }

    inline V3 getTotalRollingAcceleration(V3 const& force, VT const& m, VT const& µ1, VT const& µ2,
                                          VT const& r, V3 const& v, V3 const& n) {
      V3 const a_n   = getNormalAcceleration(force, m, n);
      V3 const a_t   = getTangentialAcceleration(force, m, µ1, µ2, r, v, n);
      V3 const a_tot = blaze::evaluate(a_n + a_t);
      return a_tot;
    }

  }   // namespace motion

  // Helpers
  inline VT getTotalFriction(const VT µ1, const VT µ2) { return µ1 * µ2; }

}   // namespace dte3607::coldet::utils::energy

#endif   // ENERGY_COMPUTATIONS_H
