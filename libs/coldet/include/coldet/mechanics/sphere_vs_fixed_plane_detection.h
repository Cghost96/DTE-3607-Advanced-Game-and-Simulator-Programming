#ifndef DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
#define DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H

#include "../bits/types.h"
#include "./compute_trajectory.h"
#include "../utils/type_conversion.h"

#include <optional>
#include <blaze/Math.h>

#include <chrono>
using namespace std::chrono_literals;

namespace dte3607::coldet::mechanics {

inline std::optional<types::ValueType> detectCollisionSphereFixedPlane(
    [[maybe_unused]] types::HighResolutionTP const& sphere_tc,
    [[maybe_unused]] types::Point3 const&           sphere_p,
    [[maybe_unused]] types::ValueType               sphere_r,
    [[maybe_unused]] types::Vector3 const&          sphere_v,
    [[maybe_unused]] types::Point3 const&           fplane_q,
    [[maybe_unused]] types::Vector3 const&          fplane_n,
    [[maybe_unused]] types::Vector3 const&          external_forces,
    [[maybe_unused]] types::HighResolutionTP const& t_0,
    [[maybe_unused]] types::Duration                timestep) {

    auto const p = sphere_p;
    auto const r = sphere_r;
    auto const q = fplane_q;
    auto const n = fplane_n;
    auto const v = sphere_v;
    auto const F = external_forces;
    auto const d = (q + r * n) - p;
    auto const current_time = sphere_tc - t_0;//utils::timeDiff(sphere_tc, t_0);
    auto const newTime = timestep - current_time;
    auto const [ds, a] = computeLinearTrajectory(v, F, newTime);
    auto const inner_ds_n = blaze::inner(ds, n);
    auto const inner_d_n = blaze::inner(d, n);

    // Handle singularities
//    if (std::abs(inner_ds_n) < 1e-5 || std::abs(inner_d_n) < 1e-5)
    if (std::abs(inner_ds_n) < 1e-5) return {std::nullopt};

    // Check collision
    auto const x = inner_d_n / inner_ds_n;
    if (x <= 1. && x > 0)
        return {x};
    else
        return {std::nullopt};
}

}   // namespace dte3607::coldet::mechanics



#endif // DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
