#ifndef DTE3607_TYPES_H
#define DTE3607_TYPES_H

#include <coldet/bits/types.h>

namespace app
{

  // Clock
  using Clock = dte3607::coldet::types::HighResolutionClock;
  using TP    = dte3607::coldet::types::HighResolutionTP;

  // Integer time types
  using Seconds      = dte3607::coldet::types::Seconds;
  using MilliSeconds = dte3607::coldet::types::MilliSeconds;
  using MicroSeconds = dte3607::coldet::types::MicroSeconds;
  using NanoSeconds  = dte3607::coldet::types::NanoSeconds;

  // Floating point time types
  using SecondsD      = dte3607::coldet::types::SecondsD;
  using MilliSecondsD = dte3607::coldet::types::MilliSecondsD;
  using MicroSecondsD = dte3607::coldet::types::MicroSecondsD;
  using NanoSecondsD  = dte3607::coldet::types::NanoSecondsD;

  // Point and Vector
  using ValueType = dte3607::coldet::types::ValueType;
  using Point3    = dte3607::coldet::types::Point3;
  using Point3H   = dte3607::coldet::types::Point3H;
  using Vector3   = dte3607::coldet::types::Vector3;
  using Vector3H  = dte3607::coldet::types::Vector3H;

}   // namespace app

#endif   // DTE3607_TYPES_H
