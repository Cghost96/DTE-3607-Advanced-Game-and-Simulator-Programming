#ifndef DTE3607_UTILS_TYPE_CONVERSION_H
#define DTE3607_UTILS_TYPE_CONVERSION_H

#include "../bits/types.h"

namespace dte3607::coldet::utils {

types::DtRep    toDt(types::Duration const& timestep);
inline types::DtRep toDt(types::Duration const& timestep) {
    return std::chrono::duration_cast<types::Dt>(timestep).count();
}

types::Duration toDuration(types::DtRep const& dt_rep);
inline types::Duration toDuration(types::DtRep const& dt_rep) {
    return std::chrono::duration_cast<types::Duration>(
               types::MicroSecondsD(dt_rep));
}

types::NanoSeconds timeDiff(types::HighResolutionTP const& t1, types::HighResolutionTP const& t2);
inline types::NanoSeconds timeDiff(types::HighResolutionTP const& t1, types::HighResolutionTP const& t2) {
    return std::chrono::duration_cast<types::NanoSeconds>(t1 - t2);
}

}   // namespace dte3607::coldet::utils

#endif // DTE3607_UTILS_TYPE_CONVERSION_H
