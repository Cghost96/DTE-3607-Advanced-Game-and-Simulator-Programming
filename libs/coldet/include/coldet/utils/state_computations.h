#ifndef STATE_COMPUTATIONS_H
#define STATE_COMPUTATIONS_H

#include <blaze/Math.h>
#include "../bits/types.h"
#include "../bits/rigidbodies.h"
#include <stdexcept>

#include <sstream>

namespace dte3607::coldet::utils::state {

  using States         = rigidbodies::Sphere::States;
  using V3             = types::Vector3;
  using VT             = types::ValueType;
  using P3             = types::Point3;
  using Sphere         = rigidbodies::Sphere;
  using FixedPlane     = rigidbodies::FixedPlane;
  using SFPAttachments = std::unordered_map<Sphere*, FixedPlane*>;
  auto const epsilon   = 1e-6;

  namespace Free {
    bool canGoToRestingState(V3 n, V3 ds);
    bool canGoToSlidingState(V3 n, V3 ds);

    inline bool canGoToRestingState(V3 n, V3 ds) {
      auto const inner_n_ds = blaze::inner(blaze::normalize(n), blaze::normalize(ds));
      return inner_n_ds >= -1 && inner_n_ds <= (-1 + epsilon);
    }

    inline bool canGoToSlidingState(V3 n, V3 ds) {
      auto const inner_ds_n = blaze::inner(ds, n);
      return inner_ds_n <= 0.;
    }
  }   // namespace Free

  namespace Resting {
    bool canGoToFreeState(V3 n, V3 ds);
    bool canGoToSlidingState(V3 n, V3 ds);

    inline bool canGoToFreeState(V3 n, V3 ds) { return blaze::inner(ds, n) > 0.; }

    inline bool canGoToSlidingState(V3 n, V3 ds) {
      auto const inner_ds_n = blaze::inner(blaze::normalize(ds), blaze::normalize(n));
      return inner_ds_n < 0 && inner_ds_n > (-1 + epsilon);
    }
  }   // namespace Resting

  namespace Sliding {
    bool canGoToFreeState(V3 n, V3 ds);
    bool canGoToRestingState(V3 n, V3 ds);

    inline bool canGoToFreeState(V3 n, V3 ds) {
      auto const inner_ds_n = blaze::inner(ds, n);
      return inner_ds_n > 0.;
    }

    inline bool canGoToRestingState(V3 n, V3 ds) {
      auto const inner_n_ds = blaze::inner(blaze::normalize(n), blaze::normalize(ds));
      return inner_n_ds >= -1 && inner_n_ds <= (-1 + epsilon);
    }
  }   // namespace Sliding

  namespace Rolling {}   // namespace Rolling

  // TODO remove cout
  void detectStateChangeOF(Sphere* o, FixedPlane* f, SFPAttachments& attachments, V3& ds) {
    switch (o->state()) {
      case States::Free: {
        bool const slidingState = Free::canGoToSlidingState(f->normal(), ds);
        bool const restingState = Free::canGoToRestingState(f->normal(), ds);
        if (slidingState && restingState) {
          o->setState(States::Resting);
          std::cout << o->m_name << " set to Resting" << std::endl;
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
        if (freeState) {
          o->setState(States::Free);
          std::cout << o->m_name << " set to Free" << std::endl;
          attachments.erase(o);
        }
        else if (slidingState) {
          o->setState(States::Sliding);
          std::cout << o->m_name << " set to Sliding" << std::endl;
        }
      } break;
      case States::Sliding: {
        bool const freeState    = Sliding::canGoToFreeState(f->normal(), ds);
        bool const restingState = Sliding::canGoToRestingState(f->normal(), ds);
        if (freeState) {
          o->setState(States::Free);
          std::cout << o->m_name << " set to Free" << std::endl;
          attachments.erase(o);
        }
        else if (restingState) {
          o->setState(States::Resting);
          std::cout << o->m_name << " set to Resting" << std::endl;
        }
      } break;
    }
  }
}   // namespace dte3607::coldet::utils::state

#endif   // STATE_COMPUTATIONS_H
