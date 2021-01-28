#ifndef DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP2_H
#define DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP2_H


#include "../concepts/scenario_fixture_concepts.h"
#include "../bits/types.h"
#include "../bits/rigidbodies.h"
#include "../mechanics/sphere_vs_fixed_plane_detection.h"
#include "../mechanics/compute_trajectory.h"
#include "../mechanics/sphere_vs_fixed_plane_response.h"
#include "../utils/type_conversion.h"

#include <memory>
#include <set>
#include <algorithm>

namespace dte3607::coldet::solver_dev::step2 {

using namespace std::chrono;

using Sphere = rigidbodies::Sphere;
using SpherePtr = std::unique_ptr<Sphere>;
using FixedPlane    = rigidbodies::FixedPlane;
using FixedPlanePtr = std::unique_ptr<FixedPlane>;
using FixedPlanes   = std::vector<FixedPlanePtr>;
using TP = types::HighResolutionTP;
using Clock = types::HighResolutionClock;
struct Collision {
    TP tp;
    Sphere* o;
    FixedPlane* f;
    bool remove;
};
using Collisions = std::vector<Collision>;

void sortAndMakeUnique(Collisions& C) {
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

    std::sort(C.begin(), C.end(),
              [](Collision c1, Collision c2) -> bool { return c1.tp < c2.tp; });
//    std::reverse(C.begin(), C.end());

    std::set<Sphere*> unique_collisions;

    for (auto it = C.rbegin(); it != C.rend(); ++it) {
        if (unique_collisions.contains(it->o)) {
            it->remove = true;
        } else {
            unique_collisions.insert(it->o);
        }
    }

    std::erase_if(C, [](const Collision & c) -> bool {
        return c.remove;
    });
}

template <typename SolverDevFixture_T>
requires concepts::scenario_fixtures::solver_dev::SolverDevFixtureStep2 <
SolverDevFixture_T > void
solve(SolverDevFixture_T& scenario,
      types::NanoSeconds  timestep) {

    // Algorithm 7 - Collision detection algorithm
    TP const    t_0 = Clock::now();
    Collisions  C   = {};
    auto const& F   = scenario.fixedPlanes();
    auto const& O   = scenario.spheres();

    for (auto& o : O) {
        o->timepoint() = t_0;
        for (auto& f : F) { // Algorithm 6 - Detect collision of a single non-fixed sphere in discretized time
            auto const collision = mechanics::detectCollisionSphereFixedPlane(
                                       o->timepoint(), o->point(), o->radius(), o->velocity(), f->point(),
                                       f->normal(), scenario.forces().G, t_0, timestep);

            if (collision) {
                auto const c_dt = std::chrono::duration_cast<types::NanoSeconds>(
                                      collision.value() * timestep);
                auto const c_tp = t_0 + c_dt;

                Collision c{c_tp, o.get(), f.get(), false};
                C.push_back(c);
            }
        }
    }

    // Algorithm 4 - sort and make unique
    sortAndMakeUnique(C);

    while(not C.empty()) {
        Collision& c = C.back();
        Sphere* s = c.o;
        FixedPlane* fp = c.f;
        C.pop_back();

        auto const [ds, a] = mechanics::computeLinearTrajectory(s->velocity(),
                             scenario.forces().G, c.tp - s->timepoint());
        s->spaceObjectFrame().translateParent(ds);
        s->addAcceleration(a);
        s->timepoint() = c.tp;
        s->setVelocity(mechanics::computeImpactResponseSphereFixedPlane(
                           s->velocity(), fp->normal()));

        // Check for possible new collisions
        for (auto& f : F) {   // Algorithm 6 - Detect collision of a single non-fixed sphere in discretized time
            // Check if the collision-object is the same as last object due to
            // floating point inaccuracy
            if(f.get() == fp)
                continue;

            auto const collision = mechanics::detectCollisionSphereFixedPlane(
                                       s->timepoint(), s->point(), s->radius(), s->velocity(), f->point(),
                                       f->normal(), scenario.forces().G, t_0, timestep);

            if (collision) {
                auto const c_dt
                    = collision.value()
                      * (timestep - (s->timepoint() - t_0)); // timepos of s within current timestep
                auto const c_dt_cast
                    = std::chrono::duration_cast<types::NanoSeconds>(c_dt);
                auto const c_tp
                    = s->timepoint() + c_dt_cast;

                Collision c_new{c_tp, s, f.get(), false};
                C.push_back(c_new);
            }
            sortAndMakeUnique(C);
        }
    }

    for(auto& sphere : scenario.spheres()) {
        auto const [ds, a] = mechanics::computeLinearTrajectory(
                                 sphere->velocity(), scenario.forces().G, timestep - (sphere->timepoint() - t_0));

        sphere->spaceObjectFrame().translateParent(ds);
        sphere->addAcceleration(a);
    }
}

}   // namespace dte3607::coldet::solver_dev::step2




#endif // DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP2_H
