#ifndef DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP4_H
#define DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP4_H


#include "../concepts/scenario_fixture_concepts.h"
#include "../bits/types.h"

namespace dte3607::coldet::solver_dev::step4
{

  template <typename SolverDevFixture_T>
  requires concepts::scenario_fixtures::solver_dev::SolverDevFixtureStep4<
    SolverDevFixture_T> void
  solve([[maybe_unused]] SolverDevFixture_T& scenario,
        [[maybe_unused]] types::NanoSeconds  timestep)
  {
  }

}   // namespace dte3607::coldet::solver_dev::step4


#endif   // DTE3607_COLDET_SOLVER_DEVELOPMENT_STEP4_H
