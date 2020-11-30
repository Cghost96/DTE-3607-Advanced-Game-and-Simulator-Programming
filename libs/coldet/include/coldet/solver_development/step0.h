#ifndef DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP0_H
#define DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP0_H


#include "../concepts/scenario_fixture_concepts.h"
#include "../bits/types.h"

#include "../mechanics/compute_trajectory.h"

namespace dte3607::coldet::solver_dev::step0 {

template <typename SolverDevFixture_T>
requires concepts::scenario_fixtures::solver_dev::SolverDevFixtureStep0 <
SolverDevFixture_T > void
solve(SolverDevFixture_T& scenario, types::NanoSeconds timestep) {
    for (auto& sphere : scenario.spheres()) {

        auto const [lin_traj, accel] = mechanics::computeLinearTrajectory(sphere->velocity(),
                                       scenario.forces().G, timestep);

        sphere->spaceObjectFrame().translateParent(lin_traj);
        sphere->addAcceleration(accel);
    }
}

}   // namespace dte3607::coldet::solver_dev::step0


#endif   // DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP0_H
