#ifndef DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H
#define DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H

#include "../bits/types.h"
#include "./compute_trajectory.h"
#include "../utils/type_conversion.h"

#include <optional>
#include <blaze/Math.h>
#include <cmath>

namespace dte3607::coldet::mechanics {

inline std::optional<types::ValueType> detectCollisionSphereSphere(
    [[maybe_unused]] types::HighResolutionTP const& s1_tc,
    [[maybe_unused]] types::Point3 const&           s1_p,
    [[maybe_unused]] types::ValueType               s1_r,
    [[maybe_unused]] types::Vector3 const&          s1_v,
    [[maybe_unused]] types::HighResolutionTP const& s2_tc,
    [[maybe_unused]] types::Point3 const&           s2_p,
    [[maybe_unused]] types::ValueType               s2_r,
    [[maybe_unused]] types::Vector3 const&          s2_v,
    [[maybe_unused]] types::Vector3 const&          external_forces,
    [[maybe_unused]] types::HighResolutionTP const& t_0,
    [[maybe_unused]] types::Duration                timestep) {

    auto const v_0 = s1_v;
    auto const v_1 = s2_v;
    auto const r_0 = s1_r;
    auto const r_1 = s2_r;
    auto const r = r_0 + r_1;
    auto const r_sq = std::pow(r, 2);
    auto const p_0 = s1_p;
    auto const p_1 = s2_p;
    auto const F = external_forces;
    auto const current_time_0 = utils::timeDiff(s1_tc, t_0);
    auto const current_time_1 = utils::timeDiff(s2_tc, t_0);
    auto const [ds_0, a_0] = computeLinearTrajectory(v_0, F, timestep - current_time_0);
    auto const [ds_1, a_1] = computeLinearTrajectory(v_1, F, timestep - current_time_1);
    auto const Q = p_1 - p_0;
    auto const R = ds_1 - ds_0;
    auto const inner_Q_R = blaze::inner(Q, R);
    auto const inner_Q_Q = blaze::inner(Q, Q);
    auto const inner_R_R = blaze::inner(R, R);
    auto const a = inner_R_R;
    auto const b = 2 * inner_Q_R;
    auto const c = inner_Q_Q - r_sq;

    // Handle singularities
    if (c < 1e-5 || a < 1e-5 || (std::pow(b, 2) - 4 * a * c) < 0)
        return {std::nullopt};

    // Check collision
    auto const x = (-b - std::sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
    if(x <= 1 && x > 0)
        return {x};
    else
        return {std::nullopt};
}


}   // namespace dte3607::coldet::mechanics



#endif   // DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H
