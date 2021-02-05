#ifndef DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H
#define DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H

#include "../bits/types.h"
#include "./compute_trajectory.h"
#include "../utils/type_conversion.h"
#include "../bits/rigidbodies.h"

#include <optional>
#include <blaze/Math.h>
#include <cmath>

namespace dte3607::coldet::mechanics {

  using Sphere       = rigidbodies::Sphere;
  using States       = rigidbodies::Sphere::States;
  using TP           = types::HighResolutionTP;
  using P3           = types::Point3;
  using V3           = types::Vector3;
  using VT           = types::ValueType;
  using DT           = types::Duration;
  using Trajectories = std::unordered_map<Sphere*, std::pair<V3, V3>>;

  inline std::optional<types::ValueType> detectCollisionSphereSphere(TP const& t1, P3 const& p1, VT r1,
                                                                     V3 const& v1, TP const& t2, P3 const& p2,
                                                                     VT r2, V3 const& v2, V3 const& F,
                                                                     TP const& t_0, DT timestep) {
    auto const epsilon        = 1e-6;
    auto const r              = r1 + r2;
    auto const r_sq           = std::pow(r, 2);
    auto const current_time_1 = utils::timeDiff(t1, t_0);
    auto const current_time_2 = utils::timeDiff(t2, t_0);
    auto const [ds1, a1]      = computeLinearTrajectory(v1, F, timestep - current_time_1);
    auto const [ds2, a2]      = computeLinearTrajectory(v2, F, timestep - current_time_2);
    auto const Q              = p2 - p1;
    auto const R              = ds2 - ds1;
    auto const inner_QR       = blaze::inner(Q, R);
    auto const inner_QQ       = blaze::inner(Q, Q);
    auto const inner_RR       = blaze::inner(R, R);
    auto const a              = inner_RR;
    auto const b              = 2 * inner_QR;
    auto const c              = inner_QQ - r_sq;

    // Handle singularities
    if (a < epsilon || (std::pow(b, 2) - 4 * a * c) < 0) return {std::nullopt};

    // Check collision
    auto const x = (-b - std::sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
    return (x <= 1. && x > 0.) ? std::make_optional(x) : std::nullopt;
  }

  inline std::optional<types::ValueType> detectCollisionSphereSphere(Sphere* o1, Sphere* o2,
                                                                     Trajectories const& trajectories) {
    auto const epsilon  = 1e-6;
    auto const r        = o1->radius() + o2->radius();
    auto const r_sq     = std::pow(r, 2);
    auto const ds1      = o1->state() == States::Resting ? V3{0, 0, 0} : trajectories.at(o1).first;
    auto const ds2      = o2->state() == States::Resting ? V3{0, 0, 0} : trajectories.at(o2).first;
    auto const Q        = o2->point() - o1->point();
    auto const R        = ds2 - ds1;
    auto const inner_QR = blaze::inner(Q, R);
    auto const inner_QQ = blaze::inner(Q, Q);
    auto const inner_RR = blaze::inner(R, R);
    auto const a        = inner_RR;
    auto const b        = 2 * inner_QR;
    auto const c        = inner_QQ - r_sq;

    // Handle singularities
    if (a < epsilon || (std::pow(b, 2) - 4 * a * c) < 0) return {std::nullopt};

    // Check collision
    auto const x = (-b - std::sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
    return (x <= 1. && x > 0.) ? std::make_optional(x) : std::nullopt;
  }

}   // namespace dte3607::coldet::mechanics



#endif   // DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H
