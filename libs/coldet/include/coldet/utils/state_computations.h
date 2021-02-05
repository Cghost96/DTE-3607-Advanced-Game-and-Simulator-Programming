#ifndef STATE_COMPUTATIONS_H
#define STATE_COMPUTATIONS_H

#include <blaze/Math.h>
#include "../bits/types.h"
#include "../bits/rigidbodies.h"

#include <sstream>

namespace dte3607::coldet::utils::state {

  using States                = rigidbodies::Sphere::States;
  using V3                    = types::Vector3;
  using VT                    = types::ValueType;
  using P3                    = types::Point3;
  using Sphere                = rigidbodies::Sphere;
  using FixedPlane            = rigidbodies::FixedPlane;
  using SFPAttachments        = std::unordered_map<Sphere*, FixedPlane*>;
  auto const epsilon          = 1e-6;
  auto const restingTolerance = 300000 * epsilon;
  auto const rollingTolerance = 1000 * epsilon;
  auto const freeTolerance    = 1000 * epsilon;

  V3 getRotationAxis(Sphere* o);

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

  inline void detectStateChange(Sphere* o, FixedPlane* f, SFPAttachments& attachments, V3& ds) {
    switch (o->state()) {
      case States::Free: {
        bool const slidingState
          = Free::canGoToSlidingState(f->normal(), ds, o->velocity(), getRotationAxis(o));
        bool const rollingState
          = Free::canGoToRollingState(f->normal(), ds, o->velocity(), getRotationAxis(o));
        bool const restingState = Free::canGoToRestingState(f->normal(), ds);
        if (restingState) {
          o->setState(States::Resting);
          std::cout << o->m_name << " set to Resting" << std::endl;
          attachments[o] = f;
        }
        else if (rollingState) {
          o->setState(States::Rolling);
          std::cout << o->m_name << " set to Rolling" << std::endl;
          attachments[o] = f;
        }
        else if (slidingState) {
          o->setState(States::Sliding);
          std::cout << o->m_name << " set to Sliding" << std::endl;
          attachments[o] = f;
        }
      } break;
      case States::Resting: {
        bool const freeState    = Resting::canGoToFreeState(f->normal(), ds);
        bool const slidingState = Resting::canGoToSlidingState(f->normal(), ds);
        bool const rollingState
          = Resting::canGoToRollingState(f->normal(), ds, o->velocity(), getRotationAxis(o));
        if (freeState) {
          o->setState(States::Free);
          std::cout << o->m_name << " set to Free" << std::endl;
          attachments.erase(o);
        }
        else if (rollingState) {
          o->setState(States::Rolling);
          std::cout << o->m_name << " set to Rolling" << std::endl;
        }
        else if (slidingState) {
          o->setState(States::Sliding);
          std::cout << o->m_name << " set to Sliding" << std::endl;
        }
      } break;
      case States::Sliding: {
        bool const freeState    = Sliding::canGoToFreeState(f->normal(), ds);
        bool const rollingState = Sliding::canGoToRollingState(o->velocity(), getRotationAxis(o));
        bool const restingState = Sliding::canGoToRestingState(f->normal(), ds);
        if (freeState) {
          o->setState(States::Free);
          std::cout << o->m_name << " set to Free" << std::endl;
          attachments.erase(o);
        }
        else if (rollingState) {
          o->setState(States::Rolling);
          std::cout << o->m_name << " set to Rolling" << std::endl;
        }
        else if (restingState) {
          o->setState(States::Resting);
          std::cout << o->m_name << " set to Resting" << std::endl;
        }
      } break;
      case States::Rolling: {
        bool freeState    = Rolling::canGoToFreeState(f->normal(), ds);
        bool slidingState = Rolling::canGoToSlidingState(o->velocity(), getRotationAxis(o));
        bool restingState = Rolling::canGoToRestingState(f->normal(), ds);
        if (freeState) {
          o->setState(States::Free);
          std::cout << o->m_name << " set to Free" << std::endl;
          attachments.erase(o);
        }
        else if (slidingState) {
          o->setState(States::Sliding);
          std::cout << o->m_name << " set to Sliding" << std::endl;
        }
        else if (restingState) {
          o->setState(States::Resting);
          std::cout << o->m_name << " set to Resting" << std::endl;
        }
      } break;
    }
  }

  // Helpers
  inline V3 getRotationAxis(Sphere* o) {
    auto const n      = o->rotationNormal();
    auto const w_axis = -(blaze::evaluate(o->radius() * blaze::evaluate(blaze::cross(o->velocity(), n))));
    return w_axis;
  }
}   // namespace dte3607::coldet::utils::state

#endif   // STATE_COMPUTATIONS_H
