#ifndef DTE3607_COLDET_MECHANICS_COMPUTE_TRAJECTORY_H
#define DTE3607_COLDET_MECHANICS_COMPUTE_TRAJECTORY_H

#include "../bits/types.h"
#include "../utils/type_conversion.h"

namespace dte3607::coldet::mechanics {

inline std::pair<types::Vector3, types::Vector3> computeLinearTrajectory(
    [[maybe_unused]] types::Vector3 const& velocity,
    [[maybe_unused]] types::Vector3 const& external_forces,
    [[maybe_unused]] types::Duration       timestep) {

    double dt = utils::toDt(timestep);
    auto lin_traj = (velocity + 0.5 * external_forces * dt);

    return {lin_traj, external_forces * dt};
}


}   // namespace dte3607::coldet::mechanics

#endif // DTE3607_COLDET_MECHANICS_COMPUTE_TRAJECTORY_H
