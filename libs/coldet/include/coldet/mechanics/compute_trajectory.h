#ifndef DTE3607_COLDET_MECHANICS_COMPUTE_TRAJECTORY_H
#define DTE3607_COLDET_MECHANICS_COMPUTE_TRAJECTORY_H

#include "../bits/types.h"
#include "../utils/type_conversion.h"

namespace dte3607::coldet::mechanics {

inline std::pair<types::Vector3, types::Vector3> computeLinearTrajectory(
    [[maybe_unused]] types::Vector3 const& velocity,
    [[maybe_unused]] types::Vector3 const& external_forces,
    [[maybe_unused]] types::Duration       timestep) {

    auto const v = velocity;
    auto const F = external_forces;
    double const dt = utils::toDt(timestep);

    auto const a = F * dt; //acceleration
    auto const ds = (v + (1.0 / 2.0) * a) * dt; //lin_traj

    return {ds, a};
}


}   // namespace dte3607::coldet::mechanics

#endif // DTE3607_COLDET_MECHANICS_COMPUTE_TRAJECTORY_H
