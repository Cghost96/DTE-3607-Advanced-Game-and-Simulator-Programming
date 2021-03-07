#ifndef STATE_COMPUTATIONS_H
#define STATE_COMPUTATIONS_H

#include <blaze/Math.h>
#include "../bits/types.h"
#include "../bits/rigidbodies.h"
#include "../utils/Instrumentor.h"

namespace dte3607::coldet::utils::state {

  using States                = rigidbodies::Sphere::States;
  using V3                    = types::Vector3;
  using VT                    = types::ValueType;
  using P3                    = types::Point3;
  using Sphere                = rigidbodies::Sphere;
  using FixedPlane            = rigidbodies::FixedPlane;
  using OsFpAttachments       = std::unordered_map<Sphere*, FixedPlane*>;
  auto const epsilon          = 1e-6;
  auto const restingTolerance = 300000 * epsilon;
  auto const rollingTolerance = 1000 * epsilon;
  auto const freeTolerance    = 1000 * epsilon;

  V3 getRotationAxis(Sphere* s);

  namespace Free {
    bool canGoToRestingState(V3 n, V3 ds);
    bool canGoToSlidingState(V3 n, V3 ds, V3 v, V3 w);
    bool canGoToRollingState(V3 n, V3 ds, V3 v, V3 w);

    inline bool canGoToRestingState(V3 n, V3 ds) {
      auto const inner_n_ds = blaze::inner(blaze::normalize(n), blaze::normalize(ds));
      return inner_n_ds >= -1 && inner_n_ds <= (-1 + restingTolerance);
    }

    inline bool canGoToSlidingState(V3 n, V3 ds, V3 v, V3 w) {
      auto const inner_ds_n = blaze::inner(blaze::normalize(ds), blaze::normalize(n));
      VT const   v_hat      = blaze::length(v) - blaze::length(w);
      return inner_ds_n <= 0. && blaze::length(v) > 0. && v_hat > rollingTolerance;
    }

    inline bool canGoToRollingState(V3 n, V3 ds, V3 v, V3 w) {
      auto const inner_ds_n = blaze::inner(blaze::normalize(ds), blaze::normalize(n));
      VT const   v_hat      = blaze::length(v) - blaze::length(w);
      return inner_ds_n <= 0. && blaze::length(v) > 0. && v_hat <= rollingTolerance;
    }
  }   // namespace Free

  namespace Resting {
    bool canGoToFreeState(V3 n, V3 ds);
    bool canGoToSlidingState(V3 n, V3 ds);
    bool canGoToRollingState(V3 n, V3 ds, V3 v, V3 w);

    inline bool canGoToFreeState(V3 n, V3 ds) {
      return blaze::inner(blaze::normalize(ds), blaze::normalize(n)) > 0.;
    }

    inline bool canGoToSlidingState(V3 n, V3 ds) {
      auto const inner_ds_n    = blaze::inner(blaze::normalize(ds), blaze::normalize(n));
      auto const is_sliding_ds = inner_ds_n < 0 && inner_ds_n > (-1 + restingTolerance);
      return is_sliding_ds;
    }

    inline bool canGoToRollingState(V3 n, V3 ds, V3 v, V3 w) {
      auto const inner_ds_n    = blaze::inner(blaze::normalize(ds), blaze::normalize(n));
      VT const   v_hat         = blaze::length(v) - blaze::length(w);
      auto const is_sliding_ds = inner_ds_n < 0 && inner_ds_n > (-1 + restingTolerance);
      return is_sliding_ds && blaze::length(v) > 0. && v_hat <= rollingTolerance;
    }
  }   // namespace Resting

  namespace Sliding {
    bool canGoToFreeState(V3 n, V3 ds);
    bool canGoToRestingState(V3 n, V3 ds);
    bool canGoToRollingState(V3 v, V3 w);

    inline bool canGoToFreeState(V3 n, V3 ds) {
      auto const inner_ds_n = blaze::inner(blaze::normalize(ds), blaze::normalize(n));
      return inner_ds_n > freeTolerance;
    }

    inline bool canGoToRestingState(V3 n, V3 ds) {
      auto const inner_n_ds = blaze::inner(blaze::normalize(n), blaze::normalize(ds));
      return inner_n_ds >= -1 && inner_n_ds <= (-1 + restingTolerance);
    }

    inline bool canGoToRollingState(V3 v, V3 w) {
      VT const v_hat = blaze::length(v) - blaze::length(w);
      return v_hat <= rollingTolerance;
    }
  }   // namespace Sliding

  namespace Rolling {
    bool canGoToFreeState(V3 n, V3 ds);
    bool canGoToSlidingState(V3 v, V3 w);
    bool canGoToRestingState(V3 n, V3 ds);

    inline bool canGoToFreeState(V3 n, V3 ds) {
      auto const inner_ds_n = blaze::inner(blaze::normalize(ds), blaze::normalize(n));
      return inner_ds_n > freeTolerance;
    }

    inline bool canGoToSlidingState(V3 v, V3 w) {
      VT const v_hat = blaze::length(v) - blaze::length(w);
      return v_hat > rollingTolerance;
    }

    inline bool canGoToRestingState(V3 n, V3 ds) {
      auto const inner_n_ds = blaze::inner(blaze::normalize(n), blaze::normalize(ds));
      return inner_n_ds >= -1 && inner_n_ds <= (-1 + restingTolerance);
    }

  }   // namespace Rolling

  inline void detectStateChange(Sphere* s, FixedPlane* p, OsFpAttachments& attachmentsOsFp, V3& ds) {
    InstrumentationTimer timer("State change");

    auto const v = s->velocity();
    auto const n = p->normal();
    switch (s->state()) {
      case States::Free: {
        bool const slidingState = Free::canGoToSlidingState(n, ds, v, getRotationAxis(s));
        bool const rollingState = Free::canGoToRollingState(n, ds, v, getRotationAxis(s));
        bool const restingState = Free::canGoToRestingState(n, ds);
        if (restingState) {
          s->setState(States::Resting);
          attachmentsOsFp[s] = p;
        }
        else if (rollingState) {
          s->setState(States::Rolling);
          attachmentsOsFp[s] = p;
        }
        else if (slidingState) {
          s->setState(States::Sliding);
          attachmentsOsFp[s] = p;
        }
      } break;
      case States::Resting: {
        bool const freeState    = Resting::canGoToFreeState(n, ds);
        bool const slidingState = Resting::canGoToSlidingState(n, ds);
        bool const rollingState = Resting::canGoToRollingState(n, ds, v, getRotationAxis(s));
        if (freeState) {
          s->setState(States::Free);
          attachmentsOsFp.erase(s);
        }
        else if (rollingState) {
          s->setState(States::Rolling);
        }
        else if (slidingState) {
          s->setState(States::Sliding);
        }
      } break;
      case States::Sliding: {
        bool const freeState    = Sliding::canGoToFreeState(n, ds);
        bool const rollingState = Sliding::canGoToRollingState(v, getRotationAxis(s));
        bool const restingState = Sliding::canGoToRestingState(n, ds);
        if (freeState) {
          s->setState(States::Free);
          attachmentsOsFp.erase(s);
        }
        else if (rollingState) {
          s->setState(States::Rolling);
        }
        else if (restingState) {
          s->setState(States::Resting);
        }
      } break;
      case States::Rolling: {
        bool freeState    = Rolling::canGoToFreeState(n, ds);
        bool slidingState = Rolling::canGoToSlidingState(v, getRotationAxis(s));
        bool restingState = Rolling::canGoToRestingState(n, ds);
        if (freeState) {
          s->setState(States::Free);
          attachmentsOsFp.erase(s);
        }
        else if (slidingState) {
          s->setState(States::Sliding);
        }
        else if (restingState) {
          s->setState(States::Resting);
        }
      } break;
    }
  }

  // Helpers
  inline V3 getRotationAxis(Sphere* s) {
    auto const n      = s->rotationNormal();
    auto const v      = s->velocity();
    auto const r      = s->radius();
    auto const w_axis = -(blaze::evaluate(r * blaze::evaluate(blaze::cross(v, n))));
    return w_axis;
  }
}   // namespace dte3607::coldet::utils::state

#endif   // STATE_COMPUTATIONS_H
