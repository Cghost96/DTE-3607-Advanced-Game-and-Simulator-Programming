#ifndef ROLLING_H
#define ROLLING_H

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

#include <blaze/Math.h>

// STL
#include <memory>
#include <set>
#include <algorithm>

// DEBUG
#include <sstream>
//#include <ctime>

namespace dte3607::coldet::mysolvers::rolling {

  using namespace std::chrono;

  using TP             = types::HighResolutionTP;
  using Clock          = types::HighResolutionClock;
  using NS             = types::NanoSeconds;
  using V3             = types::Vector3;
  using VT             = types::ValueType;
  using States         = rigidbodies::Sphere::States;
  using Sphere         = rigidbodies::Sphere;
  using SpherePtr      = std::unique_ptr<Sphere>;
  using Spheres        = std::vector<SpherePtr>;
  using FixedPlane     = rigidbodies::FixedPlane;
  using FixedPlanePtr  = std::unique_ptr<FixedPlane>;
  using FixedPlanes    = std::vector<FixedPlanePtr>;
  using SFPAttachments = std::unordered_map<Sphere*, FixedPlane*>;
  using Trajectories   = std::unordered_map<Sphere*, std::pair<V3, V3>>;
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

  void detectInitialCollisions(Spheres const& O, FixedPlanes const& F, Trajectories const& trajectories,
                               CollisionsOO& C_OO, CollisionsOF& C_OF, TP const& t_0, NS const& timestep) {
    // TODO don't detect for spheres beyond a certain distance
    for (auto& o : O) {
      // TODO don't have to detect for same spheres twice (o1 - o2 and later o2 - o1)
      for (auto& o2 : O) {
        if (o.get() == o2.get()) continue;
        auto const collision = mechanics::detectCollisionSphereSphere(o.get(), o2.get(), trajectories);
        if (collision) {
          auto const  c_dt = std::chrono::duration_cast<NS>(collision.value() * timestep);
          auto const  c_tp = t_0 + c_dt;
          CollisionOO c_oo{c_tp, o.get(), o2.get(), false};
          C_OO.push_back(c_oo);
        }
      }

      for (auto& f : F) {
        auto const collision = mechanics::detectCollisionSphereFixedPlane(o.get(), f.get(), trajectories);
        if (collision) {
          auto const  c_dt = std::chrono::duration_cast<NS>(collision.value() * timestep);
          auto const  c_tp = t_0 + c_dt;
          CollisionOF c_of{c_tp, o.get(), f.get(), false};
          C_OF.push_back(c_of);
        }
      }
    }
  }

  void detectNewCollisions(Sphere* o1, Sphere* o2, Trajectories const& trajectories, CollisionsOO& C_OO,
                           TP const& t_0, NS const& timestep) {
    auto const collision = mechanics::detectCollisionSphereSphere(o1, o2, trajectories);
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

  void detectNewCollisions(Sphere* o, FixedPlane* const& f, Trajectories const& trajectories,
                           CollisionsOF& C_OF, TP const& t_0, NS const& timestep) {
    auto const collision = mechanics::detectCollisionSphereFixedPlane(o, f, trajectories);
    if (collision) {
      auto const  c_dt      = collision.value() * (timestep - (o->timepoint() - t_0));
      auto const  c_dt_cast = std::chrono::duration_cast<NS>(c_dt);
      auto const  c_tp      = o->timepoint() + c_dt_cast;
      CollisionOF c_new{c_tp, o, f, false};
      C_OF.push_back(c_new);
    }
  }

  void setRotationNormal(Sphere* o, V3 const n) { o->setRotationNormal(blaze::normalize(n)); }

  V3 getRotationAxis(Sphere* o) {
    V3 const n      = o->rotationNormal();
    V3 const w_axis = -(blaze::evaluate(o->radius() * blaze::evaluate(blaze::cross(o->velocity(), n))));
    return w_axis;
  }

  void setRotationSpeed(Sphere* o) {
    V3 const w_axis  = getRotationAxis(o);
    VT const w_speed = blaze::evaluate(blaze::length(w_axis)) * (std::numbers::pi / 180.);
    o->setRotationSpeed(w_speed);
  }

  void simulate(Sphere* o, SFPAttachments const& attachments, Trajectories const& trajectories,
                NS const simulation_dt, NS const& full_timestep, TP const& t_0) {
    if (o->state() != States::Resting) {
      auto const remaining_dt = utils::toDt((t_0 + full_timestep) - o->timepoint());
      if (remaining_dt > 0.) {
        auto const scaling        = utils::toDt(simulation_dt) / remaining_dt;
        auto       ds             = trajectories.at(o).first * scaling;
        auto       a              = trajectories.at(o).second * scaling;
        auto const rotSpeedScaled = o->rotationSpeed() * utils::toDt(simulation_dt);
        o->spaceObjectFrame().rotateParent(rotSpeedScaled, getRotationAxis(o));
        o->spaceObjectFrame().translateParent(ds);
        if (o->state() != States::Rolling) o->addAcceleration(a);
        utils::energy::dampening::addTimeDependentLoss(o, attachments, simulation_dt);
        if (o->state() != States::Free) setRotationSpeed(o);
      }
    }
  }

  void cache(Sphere* o, TP const new_tp) { o->timepoint() = new_tp; }

  void cache(Sphere* o, V3 const ds, V3 const a, Trajectories& trajectories) {
    trajectories[o] = std::make_pair(ds, a);
  }

  void response(CollisionOO& c, SFPAttachments& attachments, Trajectories& trajectories, V3 const force,
                NS const& timestep, TP const& t_0) {
    auto const dt_next            = timestep - (c.tp - t_0);
    auto const µ1                 = c.o1->frictionCoef();
    auto const µ2                 = c.o2->frictionCoef();
    V3 const   collision_normal_1 = c.o1->point() - c.o2->point();
    V3 const   collision_normal_2 = c.o2->point() - c.o1->point();
    auto [v1_adj, v1_orig, v2_adj, v2_orig]
      = mechanics::computeImpactResponseSphereSphereWithStates(c.o1, c.o2, attachments, µ1, µ2);

    auto handle_response = [force, &attachments, &dt_next, &trajectories](Sphere* o, V3 const v_adj,
                                                                          V3 const v_orig, V3 col_n) -> void {
      auto [ds_next_free, a_next_free] = mechanics::computeLinearTrajectory(v_orig, force, dt_next);
      if (o->state() == States::Free) {
        o->setVelocity(v_orig);
        setRotationNormal(o, col_n);
        setRotationSpeed(o);
        cache(o, ds_next_free, a_next_free, trajectories);
      }
      else {
        utils::state::detectStateChange(o, attachments.at(o), attachments, ds_next_free);
        if (o->state() == States::Free) {
          o->setVelocity(v_orig);
          setRotationNormal(o, col_n);
          setRotationSpeed(o);
          cache(o, ds_next_free, a_next_free, trajectories);
        }
        else if (o->state() == States::Sliding || o->state() == States::Rolling) {
          o->setVelocity(v_adj);
          setRotationNormal(o, attachments.at(o)->normal());
          setRotationSpeed(o);
          utils::state::detectStateChange(o, attachments.at(o), attachments, ds_next_free);
          auto [ds_next_adj, a_next_adj] = mechanics::computeLinearTrajectory(o, attachments, force, dt_next);
          cache(o, ds_next_adj, a_next_adj, trajectories);
        }
      }
    };

    handle_response(c.o1, v1_adj, v1_orig, collision_normal_1);
    handle_response(c.o2, v2_adj, v2_orig, collision_normal_2);
  }

  void response(CollisionOF& c, SFPAttachments& attachments, Trajectories& trajectories, V3 const force,
                NS const& timestep, TP const& t_0) {
    auto const dt_next = timestep - (c.tp - t_0);

    auto const v_init_response       = mechanics::computeImpactResponseSphereFixedPlane(c.o, c.f);
    auto [ds_next_free, a_next_free] = mechanics::computeLinearTrajectory(v_init_response, force, dt_next);

    auto const   plane      = c.o->state() == States::Free ? c.f : attachments.at(c.o);
    States const prev_state = c.o->state();
    utils::state::detectStateChange(c.o, plane, attachments, ds_next_free);

    c.o->setVelocity(c.o->state() != prev_state ? mechanics::computeImpactResponseSphereFixedPlane(c.o, c.f)
                                                : v_init_response);
    setRotationNormal(c.o, plane->normal());
    setRotationSpeed(c.o);
    utils::state::detectStateChange(c.o, plane, attachments, ds_next_free);

    if (c.o->state() == States::Free)
      cache(c.o, ds_next_free, a_next_free, trajectories);
    else if (c.o->state() == States::Sliding || c.o->state() == States::Rolling) {
      auto [ds_next_adj, a_next_adj] = mechanics::computeLinearTrajectory(c.o, attachments, force, dt_next);
      cache(c.o, ds_next_adj, a_next_adj, trajectories);
    }
  }

  // TODO don't detect for spheres beyond a certain distance
  void searchForNewCollisions(CollisionOO& c, Spheres const& O, FixedPlanes const& F,
                              Trajectories const& trajectories, CollisionsOO& C_OO, CollisionsOF C_OF,
                              TP const& t_0, NS const& timestep) {
    for (auto& o_other : O) {
      if (c.o1 != o_other.get()) detectNewCollisions(c.o1, o_other.get(), trajectories, C_OO, t_0, timestep);
      if (c.o2 != o_other.get()) detectNewCollisions(c.o2, o_other.get(), trajectories, C_OO, t_0, timestep);
    }
    for (auto& f : F) {
      detectNewCollisions(c.o1, f.get(), trajectories, C_OF, t_0, timestep);
      detectNewCollisions(c.o2, f.get(), trajectories, C_OF, t_0, timestep);
    }
  }

  void searchForNewCollisions(CollisionOF& c, Spheres const& O, FixedPlanes const& F,
                              Trajectories const& trajectories, CollisionsOF& C_OF, CollisionsOO& C_OO,
                              TP const& t_0, NS const& timestep) {
    for (auto& o_other : O)
      if (c.o != o_other.get()) detectNewCollisions(c.o, o_other.get(), trajectories, C_OO, t_0, timestep);
    for (auto& f : F)
      if (c.f != f.get()) detectNewCollisions(c.o, f.get(), trajectories, C_OF, t_0, timestep);
  }

  void handleCollision(CollisionsOO& C_OO, CollisionsOF& C_OF, SFPAttachments& attachments,
                       Trajectories& trajectories, V3 const force, NS const& timestep, TP const& t_0,
                       Spheres const& O, FixedPlanes const& F) {
    CollisionOO c = C_OO.back();
    C_OO.pop_back();
    if (not C_OF.empty()) C_OF.pop_back();
    auto const dt1 = c.tp - c.o1->timepoint();
    auto const dt2 = c.tp - c.o2->timepoint();
    simulate(c.o1, attachments, trajectories, dt1, timestep, t_0);
    simulate(c.o2, attachments, trajectories, dt2, timestep, t_0);
    response(c, attachments, trajectories, force, timestep, t_0);
    cache(c.o1, c.tp);
    cache(c.o2, c.tp);
    searchForNewCollisions(c, O, F, trajectories, C_OO, C_OF, t_0, timestep);
  }

  void handleCollision(CollisionsOF& C_OF, CollisionsOO& C_OO, SFPAttachments& attachments,
                       Trajectories& trajectories, V3 const force, NS const& timestep, TP const& t_0,
                       Spheres const& O, FixedPlanes const& F) {
    CollisionOF c = C_OF.back();
    C_OF.pop_back();
    if (not C_OO.empty()) C_OO.pop_back();
    auto const dt = c.tp - c.o->timepoint();
    simulate(c.o, attachments, trajectories, dt, timestep, t_0);
    response(c, attachments, trajectories, force, timestep, t_0);
    cache(c.o, c.tp);
    searchForNewCollisions(c, O, F, trajectories, C_OF, C_OO, t_0, timestep);
  }

  // TODO use something else than ".contains" and "insert"? / unordered_set / unordered_map?
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
  requires concepts::scenario_fixtures::SolverFixtureGaltonRolling<SolverDevFixture_T> void
  solve(SolverDevFixture_T& scenario, NS timestep) {

    /*//DEBUG RENDER TIME
     auto now1 = std::chrono::system_clock::now();
     auto timeDuration1 = now1.time_since_epoch();
     auto t1 =
     std::chrono::duration_cast<std::chrono::NS>(timeDuration1);*/

    TP const           t_0          = Clock::now();
    CollisionsOF       C_OF         = {};
    CollisionsOO       C_OO         = {};
    FixedPlanes const& F            = scenario.fixedPlanes();
    Spheres const&     O            = scenario.spheres();
    SFPAttachments&    attachments  = scenario.sfpAttachments();
    Trajectories&      trajectories = scenario.trajectories();
    V3 const&          G            = scenario.forces().G;

    for (auto& o : O) {
      cache(o.get(), t_0);
      if (!trajectories.contains(o.get())) {
        auto [ds_next, a_next] = mechanics::computeLinearTrajectory(o->velocity(), G, timestep);
        cache(o.get(), ds_next, a_next, trajectories);
      }
    }

    detectInitialCollisions(O, F, trajectories, C_OO, C_OF, t_0, timestep);

    sortAndMakeUnique(C_OF, C_OO);

    while (not C_OF.empty() || not C_OO.empty()) {
      if (not C_OF.empty() && not C_OO.empty()) {
        if (C_OF.back().tp < C_OO.back().tp)
          handleCollision(C_OF, C_OO, attachments, trajectories, G, timestep, t_0, O, F);
        else
          handleCollision(C_OO, C_OF, attachments, trajectories, G, timestep, t_0, O, F);
      }
      else if (not C_OF.empty())
        handleCollision(C_OF, C_OO, attachments, trajectories, G, timestep, t_0, O, F);
      else
        handleCollision(C_OO, C_OF, attachments, trajectories, G, timestep, t_0, O, F);

      sortAndMakeUnique(C_OF, C_OO);
    }

    // Remaining dt
    for (auto& o : O) {
      auto const dt = timestep - (o->timepoint() - t_0);
      simulate(o.get(), attachments, trajectories, dt, timestep, t_0);
      auto [ds_next_free, a_next_free] = mechanics::computeLinearTrajectory(o->velocity(), G, timestep);

      if (o->state() == States::Free)
        cache(o.get(), ds_next_free, a_next_free, trajectories);
      else if (o->state() == States::Sliding || o->state() == States::Rolling) {
        auto const plane = attachments.at(o.get());
        utils::state::detectStateChange(o.get(), plane, attachments, ds_next_free);

        if (o->state() == States::Sliding || o->state() == States::Rolling) {
          auto [ds_next_adj, a_next_adj]
            = mechanics::computeLinearTrajectory(o.get(), attachments, G, timestep);
          cache(o.get(), ds_next_adj, a_next_adj, trajectories);
        }
        else if (o->state() == States::Free)
          cache(o.get(), ds_next_free, a_next_free, trajectories);
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
}   // namespace dte3607::coldet::mysolvers::rolling

#endif   // ROLLING_H
