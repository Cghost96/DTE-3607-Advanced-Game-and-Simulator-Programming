#ifndef DTE3607_UTILS_TYPE_CONVERSION_H
#define DTE3607_UTILS_TYPE_CONVERSION_H

#include "../bits/types.h"

namespace dte3607::coldet::utils
{

  types::DtRep    toDt(types::Duration const& timestep);
  inline types::DtRep toDt(types::Duration const& timestep)
  {
    return std::chrono::duration_cast<types::Dt>(timestep).count();
  }

  types::Duration toDuration(types::DtRep const& dt_rep);
  inline types::Duration toDuration(types::DtRep const& dt_rep)
  {
    return std::chrono::duration_cast<types::Duration>(
      types::MicroSecondsD(dt_rep));
  }

}   // namespace dte3607::coldet::utils

#endif // DTE3607_UTILS_TYPE_CONVERSION_H
