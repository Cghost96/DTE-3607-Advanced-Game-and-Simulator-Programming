#ifndef DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP3B_H
#define DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP3B_H

// Concepts
#include "../concepts/scenario_fixture_concepts.h"

// Bits
#include "../bits/types.h"
#include "../bits/rigidbodies.h"

// Mechanics
#include "../mechanics/compute_trajectory.h"
#include "../mechanics/sphere_vs_fixed_plane_detection.h"
#include "../mechanics/sphere_vs_sphere_detection.h"
#include "../mechanics/sphere_vs_fixed_plane_response.h"
#include "../mechanics/sphere_vs_sphere_response.h"

// Utils
#include "../utils/type_conversion.h"
#include "../utils/state_computations.h"
#include "../utils/energy_computations.h"

// STL
#include <memory>
#include <set>
#include <algorithm>

// DEBUG RENDER TIME
//#include <sstream>
//#include <ctime>

namespace dte3607::coldet::solver_dev::step3b {

  using namespace std::chrono;

  using States         = rigidbodies::Sphere::States;
  using Sphere         = rigidbodies::Sphere;
  using SpherePtr      = std::unique_ptr<Sphere>;
  using Spheres        = std::vector<SpherePtr>;
  using FixedPlane     = rigidbodies::FixedPlane;
  using FixedPlanePtr  = std::unique_ptr<FixedPlane>;
  using FixedPlanes    = std::vector<FixedPlanePtr>;
  using SFPAttachments = std::unordered_map<Sphere*, FixedPlane*>;
  using TP             = types::HighResolutionTP;
  using Clock          = types::HighResolutionClock;
  using NS             = types::NanoSeconds;
  using V3             = types::Vector3;
  struct CollisionOO {
    TP      tp;
    Sphere* o1;
    Sphere* o2;
    bool    remove;
  };
  struct CollisionOF {
    TP          tp;
    Sphere*     o;
    FixedPlane* f;
    bool        remove;
  };
  using CollisionsOF = std::vector<CollisionOF>;
  using CollisionsOO = std::vector<CollisionOO>;

  void detectInitialCollisions(Spheres const& O, FixedPlanes const& F, CollisionsOO& C_OO, CollisionsOF& C_OF,
                               V3 const force, TP const& t_0, NS const& timestep) {
    for (auto& o : O) {
      for (auto& o2 : O) {
        if (o.get() == o2.get()) continue;
        auto const collision = mechanics::detectCollisionSphereSphere(
          o->timepoint(), o->point(), o->radius(), o->velocity(), o2->timepoint(), o2->point(), o2->radius(),
          o2->velocity(), force, t_0, timestep);
        if (collision) {
          auto const  c_dt = std::chrono::duration_cast<NS>(collision.value() * timestep);
          auto const  c_tp = t_0 + c_dt;
          CollisionOO c_oo{c_tp, o.get(), o2.get(), false};
          C_OO.push_back(c_oo);
        }
      }

      for (auto& f : F) {
        auto const collision
          = mechanics::detectCollisionSphereFixedPlane(o->timepoint(), o->point(), o->radius(), o->velocity(),
                                                       f->point(), f->normal(), force, t_0, timestep);
        if (collision) {
          auto const  c_dt = std::chrono::duration_cast<NS>(collision.value() * timestep);
          auto const  c_tp = t_0 + c_dt;
          CollisionOF c_of{c_tp, o.get(), f.get(), false};
          C_OF.push_back(c_of);
        }
      }
    }
  }

  void detectNewCollisionsOO(Sphere* o1, Sphere* o2, CollisionsOO& C_OO, V3 const force, TP const& t_0,
                             NS const& timestep) {
    auto const collision = mechanics::detectCollisionSphereSphere(
      o1->timepoint(), o1->point(), o1->radius(), o1->velocity(), o2->timepoint(), o2->point(), o2->radius(),
      o2->velocity(), force, t_0, timestep);
    if (collision) {
      auto const c_dt      = collision.value() * (timestep - (o1->timepoint() - t_0));
      auto const c_dt_cast = std::chrono::duration_cast<NS>(c_dt);
      auto const c_tp      = o1->timepoint() + c_dt_cast;

      auto t_c_min = std::max(o1->timepoint(), o2->timepoint());
      if (c_tp >= t_c_min && c_tp < (t_0 + timestep)) {
        CollisionOO c_new{c_tp, o1, o2, false};
        C_OO.push_back(c_new);
      }
    }
  }

  void detectNewCollisionsOF(Sphere* o, FixedPlane* const& f, CollisionsOF& C_OF, V3 const force,
                             TP const& t_0, NS const& timestep) {
    auto const collision = mechanics::detectCollisionSphereFixedPlane(
      o->timepoint(), o->point(), o->radius(), o->velocity(), f->point(), f->normal(), force, t_0, timestep);
    if (collision) {
      auto const  c_dt      = collision.value() * (timestep - (o->timepoint() - t_0));
      auto const  c_dt_cast = std::chrono::duration_cast<NS>(c_dt);
      auto const  c_tp      = o->timepoint() + c_dt_cast;
      CollisionOF c_new{c_tp, o, f, false};
      C_OF.push_back(c_new);
    }
  }

  void simulate(Sphere* o, SFPAttachments const& attachments, NS const dt, V3 const force) {
    auto [ds, a] = mechanics::computeLinearTrajectory(o, attachments, force, dt);
    o->spaceObjectFrame().translateParent(ds);
    o->addAcceleration(a);
    utils::energy::dampening::addTimeDependentLoss(o, attachments, dt);
  }

  void responseOO(CollisionOO& c, SFPAttachments& attachments, V3 const force, NS const& timestep,
                  TP const& t_0) {
    auto const dt_next = timestep - (c.tp - t_0);
    auto const µ1      = c.o1->frictionCoef();
    auto const µ2      = c.o2->frictionCoef();
    auto [v1_adj, v1_orig, v2_adj, v2_orig]
      = mechanics::computeImpactResponseSphereSphereWithStates(c.o1, c.o2, attachments, µ1, µ2);

    auto handle_response = [force, &attachments, &dt_next](Sphere* o, V3 v_adj, V3 v_orig) -> void {
      if (o->state() != States::Free) {
        auto [ds_next, a_next] = mechanics::computeLinearTrajectory(v_orig, force, dt_next);
        utils::state::detectStateChange(o, attachments.at(o), attachments, ds_next);
        o->setVelocity((o->state() == States::Free) ? v_orig : v_adj);
      }
      else
        o->setVelocity(v_orig);
    };

    handle_response(c.o1, v1_adj, v1_orig);
    handle_response(c.o2, v2_adj, v2_orig);
  }

  void responseOF(CollisionOF& c, SFPAttachments& attachments, V3 const force, NS const& timestep,
                  TP const& t_0) {
    auto const   dt_next    = timestep - (c.tp - t_0);
    auto const   plane      = c.o->state() == States::Free ? c.f : attachments.at(c.o);
    auto const   response   = mechanics::computeImpactResponseSphereFixedPlane(c.o, c.f);
    States const prev_state = c.o->state();
    auto [ds_next, a_next]  = mechanics::computeLinearTrajectory(response, force, dt_next);
    utils::state::detectStateChange(c.o, plane, attachments, ds_next);
    c.o->setVelocity(c.o->state() != prev_state ? mechanics::computeImpactResponseSphereFixedPlane(c.o, c.f)
                                                : response);
  }

  void cache(Sphere* o, TP const new_tp) { o->timepoint() = new_tp; }

  void searchForNewCollisionsOO(CollisionOO& c, Spheres const& O, FixedPlanes const& F, CollisionsOO& C_OO,
                                CollisionsOF C_OF, V3 const force, TP const& t_0, NS const& timestep) {
    for (auto& o_other : O) {
      if (c.o1 != o_other.get()) detectNewCollisionsOO(c.o1, o_other.get(), C_OO, force, t_0, timestep);
      if (c.o2 != o_other.get()) detectNewCollisionsOO(c.o2, o_other.get(), C_OO, force, t_0, timestep);
    }
    for (auto& f : F) {
      detectNewCollisionsOF(c.o1, f.get(), C_OF, force, t_0, timestep);
      detectNewCollisionsOF(c.o2, f.get(), C_OF, force, t_0, timestep);
    }
  }

  void searchForNewCollisionsOF(CollisionOF& c, Spheres const& O, FixedPlanes const& F, CollisionsOF& C_OF,
                                CollisionsOO& C_OO, V3 const force, TP const& t_0, NS const& timestep) {
    for (auto& o_other : O)
      if (c.o != o_other.get()) detectNewCollisionsOO(c.o, o_other.get(), C_OO, force, t_0, timestep);
    for (auto& f : F)
      if (c.f != f.get()) detectNewCollisionsOF(c.o, f.get(), C_OF, force, t_0, timestep);
  }

  void handleCollisionOO(CollisionsOO& C_OO, CollisionsOF& C_OF, SFPAttachments& attachments, V3 const force,
                         NS const& timestep, TP const& t_0, Spheres const& O, FixedPlanes const& F) {
    CollisionOO c = C_OO.back();
    C_OO.pop_back();
    if (not C_OF.empty()) C_OF.pop_back();
    auto const dt1 = c.tp - c.o1->timepoint();
    auto const dt2 = c.tp - c.o2->timepoint();
    simulate(c.o1, attachments, dt1, force);
    simulate(c.o2, attachments, dt2, force);
    responseOO(c, attachments, force, timestep, t_0);
    cache(c.o1, c.tp);
    cache(c.o2, c.tp);
    searchForNewCollisionsOO(c, O, F, C_OO, C_OF, force, t_0, timestep);
  }

  void handleCollisionOF(CollisionsOF& C_OF, CollisionsOO& C_OO, SFPAttachments& attachments, V3 const force,
                         NS const& timestep, TP const& t_0, Spheres const& O, FixedPlanes const& F) {
    CollisionOF c = C_OF.back();
    C_OF.pop_back();
    if (not C_OO.empty()) C_OO.pop_back();
    auto const dt = c.tp - c.o->timepoint();
    simulate(c.o, attachments, dt, force);
    responseOF(c, attachments, force, timestep, t_0);
    cache(c.o, c.tp);
    searchForNewCollisionsOF(c, O, F, C_OF, C_OO, force, t_0, timestep);
  }

  // TODO use something else than ".contains" and "insert"? / unordered_set?
  void sortAndMakeUnique(CollisionsOF& C_OF, CollisionsOO& C_OO) {
    std::sort(C_OF.begin(), C_OF.end(),
              [](CollisionOF c_of1, CollisionOF c_of2) -> bool { return c_of1.tp < c_of2.tp; });
    std::sort(C_OO.begin(), C_OO.end(),
              [](CollisionOO c_oo1, CollisionOO c_oo2) -> bool { return c_oo1.tp < c_oo2.tp; });

    std::set<Sphere*>                     unique_cols_OF;
    std::set<std::pair<Sphere*, Sphere*>> unique_cols_OO;

    for (auto c_of = C_OF.rbegin(); c_of != C_OF.rend(); ++c_of) {
      if (unique_cols_OF.contains(c_of->o))
        c_of->remove = true;
      else
        unique_cols_OF.insert(c_of->o);
    }

    for (auto c_oo = C_OO.rbegin(); c_oo != C_OO.rend(); ++c_oo) {
      if (unique_cols_OO.contains(std::make_pair(c_oo->o1, c_oo->o2))
          || unique_cols_OO.contains(std::make_pair(c_oo->o2, c_oo->o1))) {
        c_oo->remove = true;
      }
      else {
        unique_cols_OO.insert(std::make_pair(c_oo->o1, c_oo->o2));
      }
    }

    std::erase_if(C_OF, [](const CollisionOF& c_of) -> bool { return c_of.remove; });
    std::erase_if(C_OO, [](const CollisionOO& c_oo) -> bool { return c_oo.remove; });
  }

  template <typename SolverDevFixture_T>
  requires concepts::scenario_fixtures::solver_dev::SolverDevFixtureStep3b<SolverDevFixture_T> void
  solve(SolverDevFixture_T& scenario, NS timestep) {

    /*//DEBUG RENDER TIME
     auto now1 = std::chrono::system_clock::now();
     auto timeDuration1 = now1.time_since_epoch();
     auto t1 =
     std::chrono::duration_cast<std::chrono::NS>(timeDuration1);*/

    TP const           t_0         = Clock::now();
    CollisionsOF       C_OF        = {};
    CollisionsOO       C_OO        = {};
    FixedPlanes const& F           = scenario.fixedPlanes();
    Spheres const&     O           = scenario.spheres();
    SFPAttachments&    attachments = scenario.sfpAttachments();
    V3 const&          G           = scenario.forces().G;

    for (auto& o : O)
      o->timepoint() = t_0;

    detectInitialCollisions(O, F, C_OO, C_OF, G, t_0, timestep);

    sortAndMakeUnique(C_OF, C_OO);

    while (not C_OF.empty() || not C_OO.empty()) {
      if (not C_OF.empty() && not C_OO.empty()) {
        if (C_OF.back().tp < C_OO.back().tp)
          handleCollisionOF(C_OF, C_OO, attachments, G, timestep, t_0, O, F);
        else
          handleCollisionOO(C_OO, C_OF, attachments, G, timestep, t_0, O, F);
      }
      else if (not C_OF.empty())
        handleCollisionOF(C_OF, C_OO, attachments, G, timestep, t_0, O, F);
      else
        handleCollisionOO(C_OO, C_OF, attachments, G, timestep, t_0, O, F);

      sortAndMakeUnique(C_OF, C_OO);
    }

    // Remaining dt
    for (auto& o : O) {
      auto const dt = timestep - (o->timepoint() - t_0);
      simulate(o.get(), attachments, dt, G);

      if (o->state() == States::Sliding) {
        auto [ds_next, a_next] = mechanics::computeLinearTrajectory(o->velocity(), G, timestep);
        auto const plane       = attachments.at(o.get());
        utils::state::detectStateChange(o.get(), plane, attachments, ds_next);
      }
    }

    /*//DEBUG RENDER TIME
    auto now2 = std::chrono::system_clock::now();
    auto timeDuration2 = now2.time_since_epoch();
    auto t2 =
    std::chrono::duration_cast<std::chrono::NS>(timeDuration2);
    if(((t2.count() - t1.count()) / 1000000) > 10.)
        std::cout << (t2.count() - t1.count()) / 1000000 << std::endl;*/
  }
}   // namespace dte3607::coldet::solver_dev::step3b

#endif   // DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP3B_H
