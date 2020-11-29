#ifndef DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H
#define DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H

#include "../bits/types.h"

namespace dte3607::coldet::mechanics
{


  inline types::Vector3 computeImpactResponseSphereFixedPlane(
    [[maybe_unused]] types::Vector3 const& sphere_v,
    [[maybe_unused]] types::Vector3 const& fplane_n)
  {
    return {};
  }


}   // namespace dte3607::coldet::mechanics


#endif // DTE3607_COLDET_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H
