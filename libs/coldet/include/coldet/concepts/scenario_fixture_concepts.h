#ifndef DTE3607_COLDET_CONCEPTS_SCENARIO_FIXTURE_CONCEPTS_H
#define DTE3607_COLDET_CONCEPTS_SCENARIO_FIXTURE_CONCEPTS_H


#include "../bits/types.h"
#include "rigidbody_concepts.h"


// clang-format off


namespace dte3607::coldet::concepts::scenario_fixtures
{

  namespace solver_dev
  {
    namespace components {

      template <typename ScenarioFixture_T>
      concept HasSpheres =
          geometry::IsSphere<typename ScenarioFixture_T::Sphere> and
          requires(ScenarioFixture_T obj) { {obj.spheres()}; } and
          requires(ScenarioFixture_T const obj) { {obj.spheres()}; };

      template <typename ScenarioFixture_T>
      concept HasFixedSpheres =
          geometry::IsSphere<typename ScenarioFixture_T::Sphere> and
          requires(ScenarioFixture_T obj) { {obj.fixedSpheres()}; } and
          requires(ScenarioFixture_T const obj) { {obj.fixedSpheres()}; };

      template <typename ScenarioFixture_T>
      concept HasAllSpheres =
              HasSpheres<ScenarioFixture_T> and
              HasFixedSpheres<ScenarioFixture_T>;

      template <typename ScenarioFixture_T>
      concept HasDynamicSpheres =
          HasSpheres<ScenarioFixture_T> and
          mechanics::Dynamic<typename ScenarioFixture_T::Sphere>;

      template <typename ScenarioFixture_T>
      concept HasTimeAwareSpheres =
          HasSpheres<ScenarioFixture_T> and
          mechanics::Timeaware<typename ScenarioFixture_T::Sphere>;

      template <typename ScenarioFixture_T>
      concept HasSpheresWithMass =
          HasSpheres<ScenarioFixture_T> and
          mechanics::HasMass<typename ScenarioFixture_T::Sphere>;

      template <typename ScenarioFixture_T>
      concept HasSpheresWithFrictionCoef =
          HasSpheres<ScenarioFixture_T> and
          mechanics::HasFrictionCoef<typename ScenarioFixture_T::Sphere>;

      template <typename ScenarioFixture_T>
      concept HasSpheresWithStates =
          HasSpheres<ScenarioFixture_T> and
          mechanics::HasStates<typename ScenarioFixture_T::Sphere> and
          mechanics::HasFreeState<typename ScenarioFixture_T::Sphere> and
          mechanics::HasRestingState<typename ScenarioFixture_T::Sphere> and
          mechanics::HasSlidingState<typename ScenarioFixture_T::Sphere> and
          mechanics::HasRollingState<typename ScenarioFixture_T::Sphere>;



      template <typename ScenarioFixture_T>
      concept HasFixedPlanes =
          geometry::IsPlane<typename ScenarioFixture_T::FixedPlane> and
          requires(ScenarioFixture_T obj) { {obj.fixedPlanes()}; } and
          requires(ScenarioFixture_T const obj) { {obj.fixedPlanes()}; };

      template <typename ScenarioFixture_T>
      concept HasFixedPlanesWithFrictionCoef =
          HasFixedPlanes<ScenarioFixture_T> and
          mechanics::HasFrictionCoef<typename ScenarioFixture_T::Sphere>;

      template <typename ScenarioFixture_T>
      concept HasExternalForces =
          requires(ScenarioFixture_T obj) { {obj.forces()}; } and
          requires(ScenarioFixture_T const obj) { {obj.forces()}; };

      template <typename ScenarioFixture_T>
      concept HasGlobalGravitation =
          requires { typename ScenarioFixture_T::Forces::G; };

      template <typename ScenarioFixture_T>
      concept HasFixedLimitedPlanes =
          geometry::IsLimitedPlane<typename ScenarioFixture_T::FixedLimitedPlane> and
          requires(ScenarioFixture_T obj) { {obj.fixedLimitedPlanes()}; } and
          requires(ScenarioFixture_T const obj) { {obj.fixedLimitedPlanes()}; };

      template <typename ScenarioFixture_T>
      concept HasFixedBezierSurfaces =
          geometry::IsBezierSurface<typename ScenarioFixture_T::FixedBezierSurface> and
          requires(ScenarioFixture_T obj) { {obj.fixedBezierSurfaces()}; } and
          requires(ScenarioFixture_T const obj) { {obj.fixedBezierSurfaces()}; };


    }   // namespace components


    template <typename ScenarioFixture_T>
    concept SolverDevFixtureStep0 =
        components::HasDynamicSpheres<ScenarioFixture_T> and
        components::HasExternalForces<ScenarioFixture_T>;

    template <typename ScenarioFixture_T>
    concept SolverDevFixtureStep1 =
        SolverDevFixtureStep0<ScenarioFixture_T> and
        components::HasFixedPlanes<ScenarioFixture_T> and
        components::HasTimeAwareSpheres<ScenarioFixture_T>;

    template <typename ScenarioFixture_T>
    concept SolverDevFixtureStep2 =
        SolverDevFixtureStep1<ScenarioFixture_T>;

    template <typename ScenarioFixture_T>
    concept SolverDevFixtureStep3a =
        SolverDevFixtureStep2<ScenarioFixture_T> and
        components::HasSpheresWithMass<ScenarioFixture_T>;

    template <typename ScenarioFixture_T>
    concept SolverDevFixtureStep3b =
        SolverDevFixtureStep2<ScenarioFixture_T> and
        components::HasSpheresWithFrictionCoef<ScenarioFixture_T> and
        components::HasSpheresWithStates<ScenarioFixture_T>;

    template <typename ScenarioFixture_T>
    concept SolverDevFixtureStep4 =
        SolverDevFixtureStep3a<ScenarioFixture_T> and
        SolverDevFixtureStep3b<ScenarioFixture_T>;

  }   // namespace solver_dev


  template <typename ScenarioFixture_T>
  concept SolverFixtureGaltonRolling =
          solver_dev::SolverDevFixtureStep4<ScenarioFixture_T>;

  template <typename ScenarioFixture_T>
  concept SolverFixtureGaltonLimitedPlane =
          SolverFixtureGaltonRolling<ScenarioFixture_T> and
          solver_dev::components::HasAllSpheres<ScenarioFixture_T>;

}   // namespace dte3607::coldet::concepts::scenario_fixtures


// clang-format on

#endif   // DTE3607_COLDET_CONCEPTS_SCENARIO_FIXTURE_CONCEPTS_H
