#ifndef LIMITED_PLANES_TIMER_H
#define LIMITED_PLANES_TIMER_H

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
#include <utility>

// DEBUG
#include <sstream>
//#include <ctime>

namespace dte3607::coldet::mysolvers::limited_planes_timer {
  using namespace std::chrono;

  using TP    = types::HighResolutionTP;
  using Clock = types::HighResolutionClock;
  using NS    = types::NanoSeconds;
  using V3    = types::Vector3;
  using VT    = types::ValueType;

  using Sphere    = rigidbodies::Sphere;
  using SpherePtr = std::unique_ptr<Sphere>;
  using Spheres   = std::vector<SpherePtr>;

  using FixedSphere    = rigidbodies::FixedSphere;
  using FixedSpherePtr = std::unique_ptr<FixedSphere>;
  using FixedSpheres   = std::vector<FixedSpherePtr>;

  using FixedPlane    = rigidbodies::FixedPlane;
  using FixedPlanePtr = std::unique_ptr<FixedPlane>;
  using FixedPlanes   = std::vector<FixedPlanePtr>;

  using FixedLimitedPlane    = rigidbodies::FixedLimitedPlane;
  using FixedLimitedPlanePtr = std::unique_ptr<FixedLimitedPlane>;
  using FixedLimitedPlanes   = std::vector<FixedLimitedPlanePtr>;

  using States          = rigidbodies::Sphere::States;
  using OsFpAttachments = std::unordered_map<Sphere*, FixedPlane*>;
  using Trajectories    = std::unordered_map<Sphere*, std::pair<V3, V3>>;

  struct CollisionOss {
    TP      tp;
    Sphere* s1;
    Sphere* s2;
    bool    remove;
  };
  struct CollisionOsFs {
    TP           tp;
    Sphere*      os;
    FixedSphere* fs;
    bool         remove;
  };
  struct CollisionOsFp {
    TP          tp;
    Sphere*     s;
    FixedPlane* p;
    bool        remove;
  };
  struct CollisionOsFlp {
    TP                 tp;
    Sphere*            s;
    FixedLimitedPlane* lp;
    bool               remove;
  };

  using CollisionsOss   = std::vector<CollisionOss>;
  using CollisionsOsFs  = std::vector<CollisionOsFs>;
  using CollisionsOsFp  = std::vector<CollisionOsFp>;
  using CollisionsOsFlp = std::vector<CollisionOsFlp>;

  void detectInitialCollisions(Spheres const& O_S, FixedSpheres const& F_S, FixedPlanes const& F_P,
                               FixedLimitedPlanes const& F_LP, Trajectories const& trajectories,
                               CollisionsOss& C_Oss, CollisionsOsFs& C_OsFs, CollisionsOsFp& C_OsFp,
                               CollisionsOsFlp& C_OsFlp, TP const& t_0, NS const& timestep, VT const& pyr_top,
                               int const s_index) {

    for (int i = 0; i < s_index; i++) {
      if (O_S[i]->state() != States::Resting) {
        for (int k = 0; k < s_index; k++) {
          if (O_S[k]->state() != States::Resting) {
            if (O_S[i].get() == O_S[k].get()) continue;
            auto const collision
              = mechanics::detectCollisionSphereSphere(O_S[i].get(), O_S[k].get(), trajectories);
            if (collision) {
              auto const   c_dt = std::chrono::duration_cast<NS>(collision.value() * timestep);
              auto const   c_tp = t_0 + c_dt;
              CollisionOss c_Oss{c_tp, O_S[i].get(), O_S[k].get(), false};
              C_Oss.push_back(c_Oss);
            }
          }
        }

        for (auto& fs : F_S) {
          if (O_S[i]->point()[1] < pyr_top + O_S[i]->radius()) {
            auto const collision
              = mechanics::detectCollisionSphereFixedSphere(O_S[i].get(), fs.get(), trajectories);
            if (collision) {
              auto const    c_dt = std::chrono::duration_cast<NS>(collision.value() * timestep);
              auto const    c_tp = t_0 + c_dt;
              CollisionOsFs c_OsFs{c_tp, O_S[i].get(), fs.get(), false};
              C_OsFs.push_back(c_OsFs);
            }
          }
        }

        for (auto& p : F_P) {
          auto const collision
            = mechanics::detectCollisionSphereFixedPlane(O_S[i].get(), p.get(), trajectories);
          if (collision) {
            auto const    c_dt = std::chrono::duration_cast<NS>(collision.value() * timestep);
            auto const    c_tp = t_0 + c_dt;
            CollisionOsFp c_OsFp{c_tp, O_S[i].get(), p.get(), false};
            C_OsFp.push_back(c_OsFp);
          }
        }

        for (auto& lp : F_LP) {
          auto const collision
            = mechanics::detectCollisionSphereFixedLimitedPlane(O_S[i].get(), lp.get(), trajectories);
          if (collision) {
            auto [x, fromSide]  = collision.value();
            auto const     c_dt = std::chrono::duration_cast<NS>(x * timestep);
            auto const     c_tp = t_0 + c_dt;
            CollisionOsFlp c_OsFlp{c_tp, O_S[i].get(), lp.get(), false};
            C_OsFlp.push_back(c_OsFlp);
          }
        }
      }
    }
  }

  void detectNewCollisions(Sphere* const s, FixedLimitedPlane* const lp, Trajectories const& trajectories,
                           CollisionsOsFlp& C_OsFlp, TP const& t_0, NS const& timestep) {
    auto const collision = mechanics::detectCollisionSphereFixedLimitedPlane(s, lp, trajectories);
    if (collision) {
      auto [x, fromSide]       = collision.value();
      auto const     c_dt      = x * (timestep - (s->timepoint() - t_0));
      auto const     c_dt_cast = std::chrono::duration_cast<NS>(c_dt);
      auto const     c_tp      = s->timepoint() + c_dt_cast;
      CollisionOsFlp c_new{c_tp, s, lp, false};
      C_OsFlp.push_back(c_new);
    }
  }

  void detectNewCollisions(Sphere* const s1, Sphere* const s2, Trajectories const& trajectories,
                           CollisionsOss& C_Oss, TP const& t_0, NS const& timestep) {
    auto const collision = mechanics::detectCollisionSphereSphere(s1, s2, trajectories);
    if (collision) {
      auto const c_dt      = collision.value() * (timestep - (s1->timepoint() - t_0));
      auto const c_dt_cast = std::chrono::duration_cast<NS>(c_dt);
      auto const c_tp      = s1->timepoint() + c_dt_cast;

      auto t_c_min = std::max(s1->timepoint(), s2->timepoint());
      if (c_tp >= t_c_min && c_tp < (t_0 + timestep)) {
        CollisionOss c_new{c_tp, s1, s2, false};
        C_Oss.push_back(c_new);
      }
    }
  }

  void detectNewCollisions(Sphere* const s, FixedSphere* const fs, Trajectories const& trajectories,
                           CollisionsOsFs& C_OsFs, TP const& t_0, NS const& timestep) {
    auto const collision = mechanics::detectCollisionSphereFixedSphere(s, fs, trajectories);
    if (collision) {
      auto const c_dt      = collision.value() * (timestep - (s->timepoint() - t_0));
      auto const c_dt_cast = std::chrono::duration_cast<NS>(c_dt);
      auto const c_tp      = s->timepoint() + c_dt_cast;

      CollisionOsFs c_new{c_tp, s, fs, false};
      C_OsFs.push_back(c_new);
    }
  }

  void detectNewCollisions(Sphere* const s, FixedPlane* const& p, Trajectories const& trajectories,
                           CollisionsOsFp& C_OsFp, TP const& t_0, NS const& timestep) {
    auto const collision = mechanics::detectCollisionSphereFixedPlane(s, p, trajectories);
    if (collision) {
      auto const    c_dt      = collision.value() * (timestep - (s->timepoint() - t_0));
      auto const    c_dt_cast = std::chrono::duration_cast<NS>(c_dt);
      auto const    c_tp      = s->timepoint() + c_dt_cast;
      CollisionOsFp c_new{c_tp, s, p, false};
      C_OsFp.push_back(c_new);
    }
  }

  void setRotationNormal(Sphere* s, V3 const n) { s->setRotationNormal(blaze::normalize(n)); }

  V3 getRotationAxis(Sphere* s) {
    V3 const n      = s->rotationNormal();
    V3 const w_axis = -(blaze::evaluate(s->radius() * blaze::evaluate(blaze::cross(s->velocity(), n))));
    return w_axis;
  }

  void setRotationSpeed(Sphere* s) {
    V3 const w_axis  = getRotationAxis(s);
    VT const w_speed = blaze::evaluate(blaze::length(w_axis)) * (std::numbers::pi / 180.);
    s->setRotationSpeed(w_speed);
  }

  void simulate(Sphere* s, OsFpAttachments const& attachmentsOsFp, Trajectories const& trajectories,
                NS const simulation_dt, NS const& full_timestep, TP const& t_0) {
    if (s->state() != States::Resting) {
      auto const remaining_dt = utils::toDt((t_0 + full_timestep) - s->timepoint());
      if (remaining_dt > 0.) {
        auto const scaling        = utils::toDt(simulation_dt) / remaining_dt;
        auto       ds             = trajectories.at(s).first * scaling;
        auto       a              = trajectories.at(s).second * scaling;
        auto const rotSpeedScaled = s->rotationSpeed() * utils::toDt(simulation_dt);
        s->spaceObjectFrame().rotateParent(rotSpeedScaled, getRotationAxis(s));
        s->spaceObjectFrame().translateParent(ds);
        if (s->state() != States::Rolling) s->addAcceleration(a);
        utils::energy::dampening::addTimeDependentLoss(s, attachmentsOsFp, simulation_dt);
        if (s->state() != States::Free) setRotationSpeed(s);
      }
    }
  }

  void cache(Sphere* s, TP const new_tp) { s->timepoint() = new_tp; }

  void cache(Sphere* s, V3 const ds, V3 const a, Trajectories& trajectories) {
    trajectories[s] = std::make_pair(ds, a);
  }

  void response(CollisionOss& c, OsFpAttachments& attachmentsOsFp, Trajectories& trajectories, V3 const force,
                NS const& timestep, TP const& t_0) {
    auto const dt_next            = timestep - (c.tp - t_0);
    auto const µ1                 = c.s1->frictionCoef();
    auto const µ2                 = c.s2->frictionCoef();
    V3 const   collision_normal_1 = c.s1->point() - c.s2->point();
    V3 const   collision_normal_2 = c.s2->point() - c.s1->point();
    auto [v1_adj, v1_orig, v2_adj, v2_orig]
      = mechanics::computeImpactResponseSphereSphereWithStates(c.s1, c.s2, attachmentsOsFp, µ1, µ2);

    auto handle_response = [force, &attachmentsOsFp, &dt_next,
                            &trajectories](Sphere* s, V3 const v_adj, V3 const v_orig, V3 col_n) -> void {
      auto [ds_next_free, a_next_free] = mechanics::computeLinearTrajectory(v_orig, force, dt_next);
      if (s->state() == States::Free) {
        s->setVelocity(v_orig);
        setRotationNormal(s, col_n);
        setRotationSpeed(s);
        cache(s, ds_next_free, a_next_free, trajectories);
      }
      else {
        utils::state::detectStateChange(s, attachmentsOsFp.at(s), attachmentsOsFp, ds_next_free);
        if (s->state() == States::Free) {
          s->setVelocity(v_orig);
          setRotationNormal(s, col_n);
          setRotationSpeed(s);
          cache(s, ds_next_free, a_next_free, trajectories);
        }
        else if (s->state() == States::Sliding || s->state() == States::Rolling) {
          s->setVelocity(v_adj);
          setRotationNormal(s, attachmentsOsFp.at(s)->normal());
          setRotationSpeed(s);
          utils::state::detectStateChange(s, attachmentsOsFp.at(s), attachmentsOsFp, ds_next_free);
          auto [ds_next_adj, a_next_adj]
            = mechanics::computeLinearTrajectory(s, attachmentsOsFp, force, dt_next);
          cache(s, ds_next_adj, a_next_adj, trajectories);
        }
      }
    };

    handle_response(c.s1, v1_adj, v1_orig, collision_normal_1);
    handle_response(c.s2, v2_adj, v2_orig, collision_normal_2);
  }

  void response(CollisionOsFs& c, Trajectories& trajectories, V3 const force, NS const& timestep,
                TP const& t_0) {
    // Spheres will always be in free state when collding with a fixed sphere
    auto const dt_next = timestep - (c.tp - t_0);

    auto const v_response = mechanics::computeImpactResponseSphereFixedSphere(c.os, c.fs);
    c.os->setVelocity(v_response);
    auto [ds_next, a_next] = mechanics::computeLinearTrajectory(v_response, force, dt_next);
    cache(c.os, ds_next, a_next, trajectories);

    auto const n = c.os->point() - c.fs->point();
    setRotationNormal(c.os, n);
    setRotationSpeed(c.os);
  }

  void response(CollisionOsFlp& c, Trajectories& trajectories, V3 const force, NS const& timestep,
                TP const& t_0) {
    // Spheres will always be in free state when colliding with a limited plane
    auto const dt_next = timestep - (c.tp - t_0);

    auto const v_response  = mechanics::computeImpactResponseSphereFixedLimitedPlane(c.s, c.lp);
    auto [ds_next, a_next] = mechanics::computeLinearTrajectory(v_response, force, dt_next);

    c.s->setVelocity(v_response);
    setRotationNormal(c.s, c.lp->normalFront());
    setRotationSpeed(c.s);

    cache(c.s, ds_next, a_next, trajectories);
  }

  void response(CollisionOsFp& c, OsFpAttachments& attachmentsOsFp, Trajectories& trajectories,
                V3 const force, NS const& timestep, TP const& t_0) {
    auto const dt_next = timestep - (c.tp - t_0);

    auto const v_init_response       = mechanics::computeImpactResponseSphereFixedPlane(c.s, c.p);
    auto [ds_next_free, a_next_free] = mechanics::computeLinearTrajectory(v_init_response, force, dt_next);

    auto const   plane      = c.s->state() == States::Free ? c.p : attachmentsOsFp.at(c.s);
    States const prev_state = c.s->state();
    utils::state::detectStateChange(c.s, plane, attachmentsOsFp, ds_next_free);

    c.s->setVelocity(c.s->state() != prev_state ? mechanics::computeImpactResponseSphereFixedPlane(c.s, c.p)
                                                : v_init_response);
    setRotationNormal(c.s, plane->normal());
    setRotationSpeed(c.s);
    utils::state::detectStateChange(c.s, plane, attachmentsOsFp, ds_next_free);

    if (c.s->state() == States::Free)
      cache(c.s, ds_next_free, a_next_free, trajectories);
    else if (c.s->state() == States::Sliding || c.s->state() == States::Rolling) {
      auto [ds_next_adj, a_next_adj]
        = mechanics::computeLinearTrajectory(c.s, attachmentsOsFp, force, dt_next);
      cache(c.s, ds_next_adj, a_next_adj, trajectories);
    }
  }

  void searchForNewCollisions(CollisionOss& c, Spheres const& O_S, FixedSpheres const& F_S,
                              FixedPlanes const& F_P, FixedLimitedPlanes const& F_LP,
                              Trajectories const& trajectories, CollisionsOss& C_Oss, CollisionsOsFs& C_OsFs,
                              CollisionsOsFp& C_OsFp, CollisionsOsFlp& C_OsFlp, TP const& t_0,
                              NS const& timestep, int const s_index) {
    for (int i = 0; i < s_index; i++) {
      if (c.s1 != O_S[i].get()) detectNewCollisions(c.s1, O_S[i].get(), trajectories, C_Oss, t_0, timestep);
      if (c.s2 != O_S[i].get()) detectNewCollisions(c.s2, O_S[i].get(), trajectories, C_Oss, t_0, timestep);
    }
    for (auto& p : F_P) {
      detectNewCollisions(c.s1, p.get(), trajectories, C_OsFp, t_0, timestep);
      detectNewCollisions(c.s2, p.get(), trajectories, C_OsFp, t_0, timestep);
    }
    for (auto& fs : F_S) {
      detectNewCollisions(c.s1, fs.get(), trajectories, C_OsFs, t_0, timestep);
      detectNewCollisions(c.s2, fs.get(), trajectories, C_OsFs, t_0, timestep);
    }
    for (auto& lp : F_LP) {
      detectNewCollisions(c.s1, lp.get(), trajectories, C_OsFlp, t_0, timestep);
      detectNewCollisions(c.s2, lp.get(), trajectories, C_OsFlp, t_0, timestep);
    }
  }

  void searchForNewCollisions(CollisionOsFs& c, Spheres const& O_S, FixedSpheres const& F_S,
                              FixedPlanes const& F_P, FixedLimitedPlanes const& F_LP,
                              Trajectories const& trajectories, CollisionsOsFs& C_OsFs,
                              CollisionsOsFp& C_OsFp, CollisionsOss& C_Oss, CollisionsOsFlp& C_OsFlp,
                              TP const& t_0, NS const& timestep, int const s_index) {
    for (int i = 0; i < s_index; i++)
      if (c.os != O_S[i].get()) detectNewCollisions(c.os, O_S[i].get(), trajectories, C_Oss, t_0, timestep);
    for (auto& p : F_P)
      detectNewCollisions(c.os, p.get(), trajectories, C_OsFp, t_0, timestep);
    for (auto& fs : F_S)
      if (c.fs != fs.get()) detectNewCollisions(c.os, fs.get(), trajectories, C_OsFs, t_0, timestep);
    for (auto& lp : F_LP)
      detectNewCollisions(c.os, lp.get(), trajectories, C_OsFlp, t_0, timestep);
  }

  void searchForNewCollisions(CollisionOsFp& c, Spheres const& O_S, FixedSpheres const& F_S,
                              FixedPlanes const& F_P, FixedLimitedPlanes const& F_LP,
                              Trajectories const& trajectories, CollisionsOsFp& C_OsFp,
                              CollisionsOsFs& C_OsFs, CollisionsOss& C_Oss, CollisionsOsFlp& C_OsFlp,
                              TP const& t_0, NS const& timestep, int const s_index) {
    for (int i = 0; i < s_index; i++)
      if (c.s != O_S[i].get()) detectNewCollisions(c.s, O_S[i].get(), trajectories, C_Oss, t_0, timestep);
    for (auto& p : F_P)
      if (c.p != p.get()) detectNewCollisions(c.s, p.get(), trajectories, C_OsFp, t_0, timestep);
    for (auto& fs : F_S)
      detectNewCollisions(c.s, fs.get(), trajectories, C_OsFs, t_0, timestep);
    for (auto& lp : F_LP)
      detectNewCollisions(c.s, lp.get(), trajectories, C_OsFlp, t_0, timestep);
  }

  void searchForNewCollisions(CollisionOsFlp& c, Spheres const& O_S, FixedLimitedPlanes const& F_LP,
                              FixedSpheres const& F_S, FixedPlanes const& F_P,
                              Trajectories const& trajectories, CollisionsOsFlp& C_OsFlp,
                              CollisionsOsFp& C_OsFp, CollisionsOsFs& C_OsFs, CollisionsOss& C_Oss,
                              TP const& t_0, NS const& timestep, int const s_index) {
    for (int i = 0; i < s_index; i++)
      if (c.s != O_S[i].get()) detectNewCollisions(c.s, O_S[i].get(), trajectories, C_Oss, t_0, timestep);
    for (auto& p : F_P)
      detectNewCollisions(c.s, p.get(), trajectories, C_OsFp, t_0, timestep);
    for (auto& fs : F_S)
      detectNewCollisions(c.s, fs.get(), trajectories, C_OsFs, t_0, timestep);
    for (auto& lp : F_LP)
      if (c.lp != lp.get()) detectNewCollisions(c.s, lp.get(), trajectories, C_OsFlp, t_0, timestep);
  }

  void handleCollision(CollisionsOss& C_Oss, CollisionsOsFp& C_OsFp, CollisionsOsFs& C_OsFs,
                       CollisionsOsFlp& C_OsFlp, OsFpAttachments& attachmentsOsFp, Trajectories& trajectories,
                       V3 const force, NS const& timestep, TP const& t_0, Spheres const& O_S,
                       FixedSpheres const& F_S, FixedPlanes const& F_P, FixedLimitedPlanes const& F_LP,
                       int const s_index) {
    CollisionOss c = C_Oss.back();
    C_Oss.pop_back();
    auto const dt1 = c.tp - c.s1->timepoint();
    auto const dt2 = c.tp - c.s2->timepoint();
    simulate(c.s1, attachmentsOsFp, trajectories, dt1, timestep, t_0);
    simulate(c.s2, attachmentsOsFp, trajectories, dt2, timestep, t_0);
    response(c, attachmentsOsFp, trajectories, force, timestep, t_0);
    cache(c.s1, c.tp);
    cache(c.s2, c.tp);
    searchForNewCollisions(c, O_S, F_S, F_P, F_LP, trajectories, C_Oss, C_OsFs, C_OsFp, C_OsFlp, t_0,
                           timestep, s_index);
  }

  void handleCollision(CollisionsOsFp& C_OsFp, [[maybe_unused]] CollisionsOss& C_Oss,
                       [[maybe_unused]] CollisionsOsFs& C_OsFs, [[maybe_unused]] CollisionsOsFlp& C_OsFlp,
                       OsFpAttachments& attachmentsOsFp, Trajectories& trajectories,
                       [[maybe_unused]] V3 const force, NS const& timestep, TP const& t_0,
                       [[maybe_unused]] Spheres const& O_S, [[maybe_unused]] FixedSpheres const& F_S,
                       [[maybe_unused]] FixedPlanes const&        F_P,
                       [[maybe_unused]] FixedLimitedPlanes const& F_LP, [[maybe_unused]] int const s_index) {
    CollisionOsFp c = C_OsFp.back();
    C_OsFp.pop_back();
    auto const dt = c.tp - c.s->timepoint();
    simulate(c.s, attachmentsOsFp, trajectories, dt, timestep, t_0);

    // Response for infinite planes turned off since the "baskets" on floor is not implemented
    c.s->setVelocity({0, 0, 0});
    c.s->setState(States::Resting);
    std::cout << "x: " << c.s->point()[0] << " z: " << c.s->point()[2] << std::endl;
    attachmentsOsFp[c.s] = c.p;
    cache(c.s, c.tp);
    //----------------------------------------------------------------------------------------

    //    response(c, attachmentsOsFp, trajectories, force, timestep, t_0);
    //    cache(c.s, c.tp);
    //    searchForNewCollisions(c, O_S, F_S, F_P, F_LP, trajectories, C_OsFp, C_OsFs, C_Oss, C_OsFlp, t_0,
    //                           timestep, s_index);
  }

  void handleCollision(CollisionsOsFs& C_OsFs, CollisionsOss& C_Oss, CollisionsOsFp& C_OsFp,
                       CollisionsOsFlp& C_OsFlp, OsFpAttachments& attachmentsOsFp, Trajectories& trajectories,
                       V3 const force, NS const& timestep, TP const& t_0, Spheres const& O_S,
                       FixedSpheres const& F_S, FixedPlanes const& F_P, FixedLimitedPlanes const& F_LP,
                       int const s_index) {
    CollisionOsFs c = C_OsFs.back();
    C_OsFs.pop_back();
    auto const dt = c.tp - c.os->timepoint();
    simulate(c.os, attachmentsOsFp, trajectories, dt, timestep, t_0);
    response(c, trajectories, force, timestep, t_0);
    cache(c.os, c.tp);
    searchForNewCollisions(c, O_S, F_S, F_P, F_LP, trajectories, C_OsFs, C_OsFp, C_Oss, C_OsFlp, t_0,
                           timestep, s_index);
  }

  void handleCollision(CollisionsOsFlp& C_OsFlp, CollisionsOss& C_Oss, CollisionsOsFp& C_OsFp,
                       CollisionsOsFs& C_OsFs, OsFpAttachments& attachmentsOsFp, Trajectories& trajectories,
                       V3 const force, NS const& timestep, TP const& t_0, Spheres const& O_S,
                       FixedSpheres const& F_S, FixedPlanes const& F_P, FixedLimitedPlanes const& F_LP,
                       int const s_index) {
    CollisionOsFlp c = C_OsFlp.back();
    C_OsFlp.pop_back();
    auto const dt = c.tp - c.s->timepoint();
    simulate(c.s, attachmentsOsFp, trajectories, dt, timestep, t_0);
    response(c, trajectories, force, timestep, t_0);
    cache(c.s, c.tp);
    searchForNewCollisions(c, O_S, F_LP, F_S, F_P, trajectories, C_OsFlp, C_OsFp, C_OsFs, C_Oss, t_0,
                           timestep, s_index);
  }

  void sortAndMakeUnique(CollisionsOsFp& C_OsFp, CollisionsOss& C_Oss, CollisionsOsFs& C_OsFs,
                         CollisionsOsFlp& C_OsFlp) {
    // Sort in descending order
    std::sort(C_OsFp.begin(), C_OsFp.end(),
              [](CollisionOsFp c1, CollisionOsFp c2) -> bool { return c1.tp > c2.tp; });
    std::sort(C_Oss.begin(), C_Oss.end(),
              [](CollisionOss c1, CollisionOss c2) -> bool { return c1.tp > c2.tp; });
    std::sort(C_OsFs.begin(), C_OsFs.end(),
              [](CollisionOsFs c1, CollisionOsFs c2) -> bool { return c1.tp > c2.tp; });
    std::sort(C_OsFlp.begin(), C_OsFlp.end(),
              [](CollisionOsFlp c1, CollisionOsFlp c2) -> bool { return c1.tp > c2.tp; });

    std::set<Sphere*>                     unique_cols_OsFp;
    std::set<std::pair<Sphere*, Sphere*>> unique_cols_Oss;
    std::set<Sphere*>                     unique_cols_OsFs;
    std::set<Sphere*>                     unique_cols_OsFlp;

    for (auto c = C_OsFp.rbegin(); c != C_OsFp.rend(); ++c) {
      if (unique_cols_OsFp.contains(c->s))
        c->remove = true;
      else
        unique_cols_OsFp.insert(c->s);
    }

    for (auto c = C_Oss.rbegin(); c != C_Oss.rend(); ++c) {
      if (unique_cols_Oss.contains(std::make_pair(c->s1, c->s2))
          || unique_cols_Oss.contains(std::make_pair(c->s2, c->s1))) {
        c->remove = true;
      }
      else {
        unique_cols_Oss.insert(std::make_pair(c->s1, c->s2));
      }
    }

    for (auto c = C_OsFs.rbegin(); c != C_OsFs.rend(); ++c) {
      if (unique_cols_OsFs.contains(c->os))
        c->remove = true;
      else
        unique_cols_OsFs.insert(c->os);
    }

    for (auto c = C_OsFlp.rbegin(); c != C_OsFlp.rend(); ++c) {
      if (unique_cols_OsFlp.contains(c->s))
        c->remove = true;
      else
        unique_cols_OsFlp.insert(c->s);
    }

    std::erase_if(C_OsFp, [](const CollisionOsFp& c) -> bool { return c.remove; });
    std::erase_if(C_Oss, [](const CollisionOss& c) -> bool { return c.remove; });
    std::erase_if(C_OsFs, [](const CollisionOsFs& c) -> bool { return c.remove; });
    std::erase_if(C_OsFlp, [](const CollisionOsFlp& c) -> bool { return c.remove; });
  }

  template <typename SolverDevFixture_T>
  requires concepts::scenario_fixtures::SolverFixtureGaltonLimitedPlane<SolverDevFixture_T> void
  solve(SolverDevFixture_T& scenario, NS timestep) {

    /*//DEBUG RENDER TIME
     auto now1 = std::chrono::system_clock::now();
     auto timeDuration1 = now1.time_since_epoch();
     auto t1 =
     std::chrono::duration_cast<std::chrono::NS>(timeDuration1);*/

    TP const         t_0             = Clock::now();
    OsFpAttachments& attachmentsOsFp = scenario.attachmentsOsFp();
    Trajectories&    trajectories    = scenario.trajectories();
    V3 const&        G               = scenario.forces().G;
    VT const&        pyr_top         = scenario.pyrTopPoint();

    scenario.time() += utils::toDt(timestep);
    if (scenario.time() >= 0.7) {
      scenario.time() = 0.;
      scenario.sIndex() += 5;
    }
    int const s_index = std::min(scenario.sIndex(), (int)scenario.spheres().size());

    Spheres const&            O_S  = scenario.spheres();
    FixedSpheres const&       F_S  = scenario.fixedSpheres();
    FixedPlanes const&        F_P  = scenario.fixedPlanes();
    FixedLimitedPlanes const& F_LP = scenario.fixedLimitedPlanes();

    CollisionsOss   C_Oss   = {};   // 1
    CollisionsOsFs  C_OsFs  = {};   // 2
    CollisionsOsFp  C_OsFp  = {};   // 3
    CollisionsOsFlp C_OsFlp = {};   // 4

    for (auto& s : O_S) {
      cache(s.get(), t_0);
      if (!trajectories.contains(s.get())) {   // only compute first frame
        auto [ds_next, a_next] = mechanics::computeLinearTrajectory(s->velocity(), G, timestep);
        cache(s.get(), ds_next, a_next, trajectories);
      }
    }

    detectInitialCollisions(O_S, F_S, F_P, F_LP, trajectories, C_Oss, C_OsFs, C_OsFp, C_OsFlp, t_0, timestep,
                            pyr_top, s_index);

    sortAndMakeUnique(C_OsFp, C_Oss, C_OsFs, C_OsFlp);

    std::vector<std::pair<TP, char>> collisions;
    while (not C_OsFp.empty() || not C_Oss.empty() || not C_OsFs.empty() || not C_OsFlp.empty()) {

      if (not C_Oss.empty()) collisions.push_back({C_Oss.back().tp, '1'});
      if (not C_OsFs.empty()) collisions.push_back({C_OsFs.back().tp, '2'});
      if (not C_OsFp.empty()) collisions.push_back({C_OsFp.back().tp, '3'});
      if (not C_OsFlp.empty()) collisions.push_back({C_OsFlp.back().tp, '4'});
      if (collisions.size() > 1) {
        std::sort(collisions.begin(), collisions.end(),
                  [](std::pair<TP, char> c1, std::pair<TP, char> c2) -> bool { return c1.first > c2.first; });
      }

      switch (collisions.back().second) {
        case '1':
          handleCollision(C_Oss, C_OsFp, C_OsFs, C_OsFlp, attachmentsOsFp, trajectories, G, timestep, t_0,
                          O_S, F_S, F_P, F_LP, s_index);
          break;
        case '2':
          handleCollision(C_OsFs, C_Oss, C_OsFp, C_OsFlp, attachmentsOsFp, trajectories, G, timestep, t_0,
                          O_S, F_S, F_P, F_LP, s_index);
          break;
        case '3':
          handleCollision(C_OsFp, C_Oss, C_OsFs, C_OsFlp, attachmentsOsFp, trajectories, G, timestep, t_0,
                          O_S, F_S, F_P, F_LP, s_index);
          break;
        case '4':
          handleCollision(C_OsFlp, C_Oss, C_OsFp, C_OsFs, attachmentsOsFp, trajectories, G, timestep, t_0,
                          O_S, F_S, F_P, F_LP, s_index);
          break;
      }
      collisions.erase(collisions.begin(), collisions.end());

      sortAndMakeUnique(C_OsFp, C_Oss, C_OsFs, C_OsFlp);
    }

    // Remaining dt
    for (int i = 0; i < s_index; i++) {
      if (O_S[i]->state() != States::Resting) {
        auto const dt = timestep - (O_S[i]->timepoint() - t_0);
        simulate(O_S[i].get(), attachmentsOsFp, trajectories, dt, timestep, t_0);
        auto [ds_next_free, a_next_free]
          = mechanics::computeLinearTrajectory(O_S[i]->velocity(), G, timestep);

        if (O_S[i]->state() == States::Free)
          cache(O_S[i].get(), ds_next_free, a_next_free, trajectories);
        else if (O_S[i]->state() == States::Sliding || O_S[i]->state() == States::Rolling) {
          auto const plane = attachmentsOsFp.at(O_S[i].get());
          utils::state::detectStateChange(O_S[i].get(), plane, attachmentsOsFp, ds_next_free);

          if (O_S[i]->state() == States::Sliding || O_S[i]->state() == States::Rolling) {
            auto [ds_next_adj, a_next_adj]
              = mechanics::computeLinearTrajectory(O_S[i].get(), attachmentsOsFp, G, timestep);
            cache(O_S[i].get(), ds_next_adj, a_next_adj, trajectories);
          }
          else if (O_S[i]->state() == States::Free)
            cache(O_S[i].get(), ds_next_free, a_next_free, trajectories);
        }
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
}   // namespace dte3607::coldet::mysolvers::limited_planes_timer

#endif   // LIMITED_PLANES_TIMER_H
