#ifndef ENERGY_COMPUTATIONS_H
#define ENERGY_COMPUTATIONS_H

#include <blaze/Math.h>
#include "../bits/types.h"
#include "../bits/rigidbodies.h"
#include "../utils/type_conversion.h"

#include <sstream>

namespace dte3607::coldet::utils::energy {

  using VT             = types::ValueType;
  using V3             = types::Vector3;
  using NS             = types::NanoSeconds;
  using Sphere         = rigidbodies::Sphere;
  using FixedPlane     = rigidbodies::FixedPlane;
  using SFPAttachments = std::unordered_map<Sphere*, FixedPlane*>;
  using States         = rigidbodies::Sphere::States;

  VT getTotalFriction(const VT µ1, const VT µ2);

  namespace dampening {
    void addTimeDependentLoss(Sphere* o, SFPAttachments const& attachments, NS const dt) {
      if (o->state() == States::Sliding) {
        auto const µ1          = o->frictionCoef();
        auto const µ2          = attachments.at(o)->frictionCoef();
        auto const dt_seconds  = utils::toDt(dt);
        VT const   loss_factor = getTotalFriction(µ1, µ2) * dt_seconds;
        V3 const   new_v
          = {o->velocity()[0] - (o->velocity()[0] * loss_factor), o->velocity()[1] - (o->velocity()[1] * loss_factor),
             o->velocity()[2] - (o->velocity()[2] * loss_factor)};

        o->setVelocity(new_v);
      }
    }

    void addTimeIndependentLoss(V3& v_response, VT const µ1, VT const µ2) {
      auto const loss_factor = getTotalFriction(µ1, µ2) * 1.;
      v_response = V3{v_response[0] - (v_response[0] * loss_factor), v_response[1] - (v_response[1] * loss_factor),
                      v_response[2] - (v_response[2] * loss_factor)};
    }
  }   // namespace dampening

  namespace force {
    V3 computeSlidingForce(V3 const F, VT const µ1, VT const µ2) { return F - (F * getTotalFriction(µ1, µ2)); }
  }   // namespace force

  // Helpers
  VT getTotalFriction(const VT µ1, const VT µ2) { return µ1 * µ2; }

}   // namespace dte3607::coldet::utils::energy

#endif   // ENERGY_COMPUTATIONS_H
