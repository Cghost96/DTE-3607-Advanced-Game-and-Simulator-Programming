#ifndef DTE3607_COLDET_MYSOLVERS_MYFIRSTSOLVER_H
#define DTE3607_COLDET_MYSOLVERS_MYFIRSTSOLVER_H

#include "../concepts/scenario_fixture_concepts.h"
#include "../bits/types.h"

namespace dte3607::coldet::mysolvers
{

  template <typename SolverDevFixture_T>
  requires concepts::scenario_fixtures::MySolverFixtureConcept01<
    SolverDevFixture_T> void
  solveSphereAndFixedPlanesNoExternalForces(
    [[maybe_unused]] SolverDevFixture_T& scenario,
    [[maybe_unused]] types::NanoSeconds  timestep)
  { }

}   // namespace dte3607::coldet::mysolvers


#endif // DTE3607_COLDET_MYSOLVERS_MYFIRSTSOLVER_H
