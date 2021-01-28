#ifndef DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP3A_H
#define DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP3A_H

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

// STL
#include <memory>
#include <set>
#include <algorithm>

/*//DEBUG TIME
#include <sstream>
#include <ctime>*/

namespace dte3607::coldet::solver_dev::step3a {

using namespace std::chrono;

using Sphere = rigidbodies::Sphere;
using SpherePtr = std::unique_ptr<Sphere>;
using Spheres = std::vector<SpherePtr>;
using FixedPlane    = rigidbodies::FixedPlane;
using FixedPlanePtr = std::unique_ptr<FixedPlane>;
using FixedPlanes   = std::vector<FixedPlanePtr>;
using TP = types::HighResolutionTP;
using Clock = types::HighResolutionClock;
using NanoSeconds = types::NanoSeconds;
using V3 = types::Vector3;
struct CollisionOO {
    TP tp;
    Sphere* o1;
    Sphere* o2;
    bool remove;
};
struct CollisionOF {
    TP tp;
    Sphere* o;
    FixedPlane* f;
    bool remove;
};
using CollisionsOF = std::vector<CollisionOF>;
using CollisionsOO = std::vector<CollisionOO>;

void detectInitialCollisions(Spheres const &O, FixedPlanes const &F,
                             V3 const &force, TP const &t_0, NanoSeconds const &timestep,
                             CollisionsOO &C_OO, CollisionsOF &C_OF) {
    for(auto& o : O) {
        for(auto& o2 : O) {
            if(o.get() == o2.get())
                continue;
            auto const collision = mechanics::detectCollisionSphereSphere(
                                       o->timepoint(), o->point(), o->radius(), o->velocity(),
                                       o2->timepoint(), o2->point(), o2->radius(), o2->velocity(),
                                       force, t_0, timestep);
            if(collision) {
                auto const c_dt = std::chrono::duration_cast<types::NanoSeconds>(collision.value() * timestep);
                auto const c_tp = t_0 + c_dt;
                CollisionOO c_oo {c_tp, o.get(), o2.get(), false};
                C_OO.push_back(c_oo);
            }
        }

        for (auto& f : F) {
            auto const collision = mechanics::detectCollisionSphereFixedPlane(
                                       o->timepoint(), o->point(), o->radius(), o->velocity(), f->point(),
                                       f->normal(), force, t_0, timestep);
            if (collision) {
                auto const c_dt = std::chrono::duration_cast<types::NanoSeconds>(collision.value() * timestep);
                auto const c_tp = t_0 + c_dt;
                CollisionOF c_of {c_tp, o.get(), f.get(), false};
                C_OF.push_back(c_of);
            }
        }
    }
}

void detectNewCollisionsOO(Sphere* o1, Sphere* o2, V3 const& force, TP const& t_0,
                           NanoSeconds const& timestep, CollisionsOO &C_OO) {
    auto const collision = mechanics::detectCollisionSphereSphere(
                               o1->timepoint(), o1->point(), o1->radius(), o1->velocity(),
                               o2->timepoint(), o2->point(), o2->radius(),
                               o2->velocity(), force, t_0, timestep);
    if (collision) {
        auto const c_dt = collision.value() * (timestep - (o1->timepoint() - t_0));
        auto const c_dt_cast = std::chrono::duration_cast<types::NanoSeconds>(c_dt);
        auto const c_tp = o1->timepoint() + c_dt_cast;

        auto t_c_min = std::max(o1->timepoint(), o2->timepoint());
        if (c_tp >= t_c_min && c_tp < (t_0 + timestep)) {
            CollisionOO c_new{c_tp, o1, o2, false};
            C_OO.push_back(c_new);
        }
    }
}

void detectNewCollisionsOF(Sphere* o, FixedPlanePtr const& f, V3 const& force, TP const& t_0,
                           NanoSeconds const& timestep, CollisionsOF &C_OF) {
    auto const collision = mechanics::detectCollisionSphereFixedPlane(
                               o->timepoint(), o->point(), o->radius(), o->velocity(),
                               f->point(), f->normal(), force, t_0, timestep);
    if (collision) {
        auto const c_dt = collision.value() * (timestep - (o->timepoint() - t_0));
        auto const c_dt_cast = std::chrono::duration_cast<types::NanoSeconds>(c_dt);
        auto const c_tp = o->timepoint() + c_dt_cast;
        CollisionOF c_new{c_tp, o, f.get(), false};
        C_OF.push_back(c_new);
    }
}

void simulateResponseAndCacheOO(CollisionOO &c, V3 const &force) {
    auto const [ds1, a1] = mechanics::computeLinearTrajectory(
                               c.o1->velocity(), force, c.tp - c.o1->timepoint());
    auto const [ds2, a2] = mechanics::computeLinearTrajectory(
                               c.o2->velocity(), force, c.tp - c.o2->timepoint());
    auto const velocities = mechanics::computeImpactResponseSphereSphere(
                                c.o1->point(), c.o1->velocity(), c.o1->mass(), c.o2->point(),
                                c.o2->velocity(), c.o2->mass());
    c.o1->spaceObjectFrame().translateParent(ds1);
    c.o1->addAcceleration(a1);
    c.o1->timepoint() = c.tp;
    c.o1->setVelocity(velocities.first);
    c.o2->spaceObjectFrame().translateParent(ds2);
    c.o2->addAcceleration(a2);
    c.o2->timepoint() = c.tp;
    c.o2->setVelocity(velocities.second);
}

void simulateResponseAndCacheOF(CollisionOF &c, V3 const &force) {
    auto const [ds, a] = mechanics::computeLinearTrajectory(c.o->velocity(), force, c.tp - c.o->timepoint());
    c.o->spaceObjectFrame().translateParent(ds);
    c.o->addAcceleration(a);
    c.o->timepoint() = c.tp;
    auto const v = mechanics::computeImpactResponseSphereFixedPlane(c.o->velocity(), c.f->normal());
    c.o->setVelocity(v);
}

void sortAndMakeUnique(CollisionsOF& C_OF, CollisionsOO& C_OO) {
    /*    struct CollisionsHasher {
             size_t operator()(const Collision& c) const
             {
               return (hash<size_t>()(c.o.get())) ^ (hash < size_t()(c.o.get()));
             }
           };

           struct CollisionsComparator {
               bool operator()(const Collision& c1, const Collision& c2) const {
                   return c1.o.get() == c2.o.get();
               }
           };

           std::sort(C.begin(), C.end(), std::greater<Collision>());
           std::unordered_set<Collision> unique_spheres;*/

    std::sort(C_OF.begin(), C_OF.end(),
              [](CollisionOF c_of1, CollisionOF c_of2) -> bool { return c_of1.tp < c_of2.tp; });
    std::sort(C_OO.begin(), C_OO.end(),
              [](CollisionOO c_oo1, CollisionOO c_oo2) -> bool { return c_oo1.tp < c_oo2.tp; });

    std::set<Sphere*> u_cols_OF;
    std::set<std::pair<Sphere*, Sphere*>> u_cols_OO;

    for (auto c_of = C_OF.rbegin(); c_of != C_OF.rend(); ++c_of) {
        if (u_cols_OF.contains(c_of->o))
            c_of->remove = true;
        else
            u_cols_OF.insert(c_of->o);
    }

    for(auto c_oo = C_OO.rbegin(); c_oo != C_OO.rend(); ++c_oo) {
        if (u_cols_OO.contains(std::make_pair(c_oo->o1, c_oo->o2)) ||
                u_cols_OO.contains(std::make_pair(c_oo->o2, c_oo->o1))) {
            c_oo->remove = true;
        } else {
            u_cols_OO.insert(std::make_pair(c_oo->o1, c_oo->o2));
        }
    }

    std::erase_if(C_OF,
                  [](const CollisionOF & c_of) -> bool { return c_of.remove; });
    std::erase_if(C_OO,
                  [](const CollisionOO & c_oo) -> bool { return c_oo.remove; });
}

template <typename SolverDevFixture_T>
requires concepts::scenario_fixtures::solver_dev::SolverDevFixtureStep3a <
SolverDevFixture_T > void
solve([[maybe_unused]] SolverDevFixture_T& scenario,
      [[maybe_unused]] types::NanoSeconds  timestep) {

    /*//DEBUG TIME
    auto now1 = std::chrono::system_clock::now();
    auto timeDuration1 = now1.time_since_epoch();
    auto t1 = std::chrono::duration_cast<std::chrono::nanoseconds>(timeDuration1);*/

    TP const           t_0  = Clock::now();
    CollisionsOF       C_OF = {};
    CollisionsOO       C_OO = {};
    FixedPlanes const& F    = scenario.fixedPlanes();
    Spheres const&     O    = scenario.spheres();

    for (auto& o : O) {
        o->timepoint() = t_0;
    }

    detectInitialCollisions(O, F, scenario.forces().G, t_0, timestep, C_OO, C_OF);

    sortAndMakeUnique(C_OF, C_OO);

    while (not C_OF.empty() || not C_OO.empty()) {
        CollisionOF c_of = {};
        CollisionOO c_oo = {};
        bool isColTypeOO = false;
        bool isColTypeOF = false;

        // Pick correct collision based on time
        if (not C_OF.empty() && not C_OO.empty()) {
            if (C_OF.back().tp < C_OO.back().tp) {
                c_of = C_OF.back();
                C_OF.pop_back();
                C_OO.pop_back();
                isColTypeOF = true;
                simulateResponseAndCacheOF(c_of, scenario.forces().G);
            } else {
                c_oo = C_OO.back();
                C_OF.pop_back();
                C_OO.pop_back();
                isColTypeOO = true;
                simulateResponseAndCacheOO(c_oo, scenario.forces().G);
            }
        } else if (not C_OF.empty()) {
            c_of = C_OF.back();
            C_OF.pop_back();
            isColTypeOF = true;
            simulateResponseAndCacheOF(c_of, scenario.forces().G);
        } else { // C_OO is not empty
            c_oo = C_OO.back();
            C_OO.pop_back();
            isColTypeOO = true;
            simulateResponseAndCacheOO(c_oo, scenario.forces().G);
        }

        // Search for possible new collisions
        if (isColTypeOO) {
            for (auto& o_other : O) {
                // o1
                if(c_oo.o1 != o_other.get())
                    detectNewCollisionsOO(c_oo.o1, o_other.get(), scenario.forces().G, t_0, timestep, C_OO);
                // o2
                if(c_oo.o2 != o_other.get())
                    detectNewCollisionsOO(c_oo.o2, o_other.get(), scenario.forces().G, t_0, timestep, C_OO);
            }
            for (auto& f : F) {
                // o1
                detectNewCollisionsOF(c_oo.o1, f, scenario.forces().G, t_0, timestep, C_OF);
                // o2
                detectNewCollisionsOF(c_oo.o2, f, scenario.forces().G, t_0, timestep, C_OF);
            }
        } else {
            for (auto& o_other : O) {
                if (c_of.o == o_other.get())
                    continue;
                detectNewCollisionsOO(c_of.o, o_other.get(), scenario.forces().G, t_0, timestep, C_OO);
            }
            for (auto& f : F) {
                if (c_of.f == f.get())
                    continue;
                detectNewCollisionsOF(c_of.o, f, scenario.forces().G, t_0, timestep, C_OF);
            }
        }
        sortAndMakeUnique(C_OF, C_OO);
    }

    // Simulate remaining dt
    for (auto& o : O) {
        auto const [ds, a] = mechanics::computeLinearTrajectory(
                                 o->velocity(), scenario.forces().G, timestep - (o->timepoint() - t_0));
        o->spaceObjectFrame().translateParent(ds);
        o->addAcceleration(a);
    }

    /*//DEBUG TIME
    auto now2 = std::chrono::system_clock::now();
    auto timeDuration2 = now2.time_since_epoch();
    auto t2 = std::chrono::duration_cast<std::chrono::nanoseconds>(timeDuration2);
    if(((t2.count() - t1.count()) / 1000000) > 10.)
        std::cout << (t2.count() - t1.count()) / 1000000 << std::endl;*/
}

}   // namespace dte3607::coldet::solver_dev::step3a


#endif // DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP3A_H
