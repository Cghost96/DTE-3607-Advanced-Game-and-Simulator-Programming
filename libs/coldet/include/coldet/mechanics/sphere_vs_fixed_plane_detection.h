#ifndef DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
#define DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H

#include "../bits/types.h"
#include "./compute_trajectory.h"
#include "../utils/type_conversion.h"
#include "../bits/rigidbodies.h"

#include <optional>
#include <blaze/Math.h>
#include <sstream>
#include <chrono>
using namespace std::chrono_literals;

namespace dte3607::coldet::mechanics {

  using Sphere            = rigidbodies::Sphere;
  using FixedPlane        = rigidbodies::FixedPlane;
  using FixedLimitedPlane = rigidbodies::FixedLimitedPlane;
  using States            = rigidbodies::Sphere::States;
  using TP                = types::HighResolutionTP;
  using P3                = types::Point3;
  using V3                = types::Vector3;
  using VT                = types::ValueType;
  using DT                = types::Duration;
  using Trajectories      = std::unordered_map<Sphere*, std::pair<V3, V3>>;

  inline std::optional<VT> detectCollisionSphereFixedPlane(TP const& tc, P3 const& p, VT r, V3 const& v,
                                                           P3 const& q, V3 const& n, V3 const& F,
                                                           TP const& t_0, DT timestep) {
    auto const epsilon      = 1e-6;
    auto const d            = (q + r * blaze::normalize(n)) - p;
    auto const current_time = tc - t_0;
    auto const newTime      = timestep - current_time;
    auto const [ds, a]      = computeLinearTrajectory(v, F, newTime);
    auto const inner_ds_n   = blaze::inner(ds, n);
    auto const inner_d_n    = blaze::inner(d, n);

    // Handle singularities
    if (std::abs(inner_ds_n) < epsilon) return {std::nullopt};

    // Check collision
    auto const x = inner_d_n / inner_ds_n;
    return (x <= 1. && x > 0.) ? std::make_optional(x) : std::nullopt;
  }

  inline std::optional<VT> detectCollisionSphereFixedPlane(Sphere* s, FixedPlane* p,
                                                           Trajectories const& trajectories) {
    auto const epsilon    = 1e-6;
    auto const d          = (p->point() + s->radius() * blaze::normalize(p->normal())) - s->point();
    auto const ds         = s->state() == States::Resting ? V3{0, 0, 0} : trajectories.at(s).first;
    auto const inner_ds_n = blaze::inner(ds, p->normal());
    auto const inner_d_n  = blaze::inner(d, p->normal());

    // Handle singularities
    if (std::abs(inner_ds_n) < epsilon) return {std::nullopt};

    // Check collision
    auto const x = inner_d_n / inner_ds_n;
    return (x <= 1. && x > 0.) ? std::make_optional(x) : std::nullopt;
  }

  inline std::optional<std::pair<VT, bool>>
  detectCollisionSphereFixedLimitedPlane(Sphere* s, FixedLimitedPlane* lp, Trajectories const& trajectories) {
    auto const epsilon = 1e-6;
    bool       initCollision;
    auto const n             = blaze::evaluate(blaze::normalize(lp->normalFront()));
    auto const d_init        = blaze::evaluate((lp->point() + s->radius() * n) - s->point());
    auto const ds            = s->state() == States::Resting ? V3{0, 0, 0} : trajectories.at(s).first;
    auto const inner_ds_n    = blaze::inner(ds, n);
    auto const inner_dinit_n = blaze::inner(d_init, n);

    // Handle singularities
    if (std::abs(inner_ds_n) < epsilon) return std::nullopt;

    // Check for initial collision (infinite plane)
    auto const x  = inner_dinit_n / inner_ds_n;
    initCollision = x <= 1. && x > 0.;

    if (not initCollision)
      return {std::nullopt};
    else {   // Check if sphere is inside plane parametrics
      auto const d     = blaze::evaluate(lp->point() - s->point());
      auto const u     = lp->uAxisLocal();
      auto const v     = lp->vAxisLocal();
      auto const u_hat = blaze::evaluate((-blaze::inner(d, blaze::normalize(u))) / (blaze::length(u)));
      auto const v_hat = blaze::evaluate((-blaze::inner(d, blaze::normalize(v))) / (blaze::length(v)));
      bool const insideParametrics = u_hat >= 0. && u_hat <= 1. && v_hat >= 0. && v_hat <= 1.;

      // TODO Not working properly???? Necessary at all (seperate collision detectio for cylinder)?
      auto const inner_ds_nf = blaze::inner(ds, lp->normalFront());
      auto const inner_ds_nb = blaze::inner(ds, lp->normalBack());
      bool const fromSide    = inner_ds_nf >= 0. && inner_ds_nb >= 0.;

      return insideParametrics ? std::make_optional(std::make_pair(x, fromSide)) : std::nullopt;
    }
  }

}   // namespace dte3607::coldet::mechanics



#endif   // DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
