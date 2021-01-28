#ifndef DTE3607_COLDET_SOLVER_DEVELOPMENT_TESTING_SOLVERS_H
#define DTE3607_COLDET_SOLVER_DEVELOPMENT_TESTING_SOLVERS_H


#include "../concepts/scenario_fixture_concepts.h"
#include "../bits/types.h"

// implementation
#include "../utils/type_conversion.h"



namespace dte3607::coldet::solver_dev::testing
{
  template <typename SolverDevFixture_T>
  void
  componentTestingSolverPassive([[maybe_unused]] SolverDevFixture_T& scenario,
                                [[maybe_unused]] types::NanoSeconds  timestep)
  {
  }

  template <typename SolverDevFixture_T>
  void componentTestingSolverRotationOfSpecificObjects01(
    SolverDevFixture_T& scenario, types::NanoSeconds timestep)
  {
    auto& spheres = scenario.spheres();
    spheres[0]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {1, 0, 0});

    spheres[1]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {0, 1, 0});

    spheres[2]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {0, 0, 1});

    spheres[3]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {1, 0, 1});

    spheres[4]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {1, 0, 0});

    spheres[5]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {0, 1, 0});

    spheres[6]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {0, 0, 1});

    spheres[7]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {1, 0, 1});

    spheres[8]->spaceObjectFrame().rotateParent(
      std::numbers::pi * utils::toDt(timestep), {1, 0, 0});

    spheres[9]->spaceObjectFrame().rotateParent(
      std::numbers::pi * utils::toDt(timestep), {0, 1, 0});

    spheres[10]->spaceObjectFrame().rotateParent(
      std::numbers::pi * utils::toDt(timestep), {0, 0, 1});

    spheres[11]->spaceObjectFrame().rotateParent(
      std::numbers::pi * utils::toDt(timestep), {1, 0, 1});



    auto& fixedPlanes = scenario.fixedPlanes();
    fixedPlanes[0]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {1, 0, 0});

    fixedPlanes[1]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {0, 1, 0});

    fixedPlanes[2]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {0, 0, 1});

    fixedPlanes[3]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {1, 0, 1});

    fixedPlanes[4]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {1, 0, 0});

    fixedPlanes[5]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {0, 1, 0});

    fixedPlanes[6]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {0, 0, 1});

    fixedPlanes[7]->spaceObjectFrame().rotateLocal(
      std::numbers::pi * utils::toDt(timestep), {1, 0, 1});

    fixedPlanes[8]->spaceObjectFrame().rotateParent(
      std::numbers::pi * utils::toDt(timestep), {1, 0, 0});

    fixedPlanes[9]->spaceObjectFrame().rotateParent(
      std::numbers::pi * utils::toDt(timestep), {0, 1, 0});

    fixedPlanes[10]->spaceObjectFrame().rotateParent(
      std::numbers::pi * utils::toDt(timestep), {0, 0, 1});

    fixedPlanes[11]->spaceObjectFrame().rotateParent(
      std::numbers::pi * utils::toDt(timestep), {1, 0, 1});
  }


}   // namespace dte3607::coldet::solver_dev::testing



#endif // DTE3607_COLDET_SOLVER_DEVELOPMENT_TESTING_SOLVERS_H
