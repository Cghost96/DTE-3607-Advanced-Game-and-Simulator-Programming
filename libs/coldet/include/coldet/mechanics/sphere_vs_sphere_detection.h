#ifndef DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H
#define DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H

#include "../bits/types.h"
#include <optional>

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
    return {};
}


}   // namespace dte3607::coldet::mechanics



#endif   // DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H
