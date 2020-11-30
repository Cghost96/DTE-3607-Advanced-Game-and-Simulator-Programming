#ifndef DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
#define DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H

#include "../bits/types.h"
#include <optional>

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
    return {};
}

}   // namespace dte3607::coldet::mechanics



#endif // DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
