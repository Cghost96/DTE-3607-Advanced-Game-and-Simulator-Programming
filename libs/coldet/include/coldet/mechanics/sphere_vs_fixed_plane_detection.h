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

  using Sphere       = rigidbodies::Sphere;
  using FixedPlane   = rigidbodies::FixedPlane;
  using States       = rigidbodies::Sphere::States;
  using TP           = types::HighResolutionTP;
  using P3           = types::Point3;
  using V3           = types::Vector3;
  using VT           = types::ValueType;
  using DT           = types::Duration;
  using Trajectories = std::unordered_map<Sphere*, std::pair<V3, V3>>;

  inline std::optional<VT> detectCollisionSphereFixedPlane(TP const& tc, P3 const& p, VT r, V3 const& v,
                                                           P3 const& q, V3 const& n, V3 const& F,
                                                           TP const& t_0, DT timestep) {
    auto const epsilon      = 1e-6;
    auto const d            = (q + r * n) - p;
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

  inline std::optional<VT> detectCollisionSphereFixedPlane(Sphere* o, FixedPlane* f,
                                                           Trajectories const& trajectories) {
    auto const epsilon    = 1e-6;
    auto const d          = (f->point() + o->radius() * f->normal()) - o->point();
    auto const ds         = o->state() == States::Resting ? V3{0, 0, 0} : trajectories.at(o).first;
    auto const inner_ds_n = blaze::inner(ds, f->normal());
    auto const inner_d_n  = blaze::inner(d, f->normal());

    // Handle singularities
    if (std::abs(inner_ds_n) < epsilon) return {std::nullopt};

    // Check collision
    auto const x = inner_d_n / inner_ds_n;
    return (x <= 1. && x > 0.) ? std::make_optional(x) : std::nullopt;
  }

}   // namespace dte3607::coldet::mechanics



#endif   // DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
