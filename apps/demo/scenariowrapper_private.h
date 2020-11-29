#ifndef DTE3607_SCENARIOWRAPPER_PRIVATE_H
#define DTE3607_SCENARIOWRAPPER_PRIVATE_H


#include "types.h"

// coldet
#include <coldet/concepts/scenario_fixture_concepts.h>

// stl
#include <optional>
#include <tuple>

namespace app::detail
{

  namespace ns
    = dte3607::coldet::concepts::scenario_fixtures::solver_dev::components;

  template <typename ScenarioWrapper_T>
  size_t getNumberOfSpheres(ScenarioWrapper_T const& /*s*/) { return 0; }
  template <typename ScenarioWrapper_T> requires ns::HasSpheres<ScenarioWrapper_T>
  size_t getNumberOfSpheres(ScenarioWrapper_T const& s)
  {
    return s.spheres().size();
  }

  template <typename ScenarioWrapper_T>
  size_t getNumberOfFixedPlanes(ScenarioWrapper_T const& /*s*/) { return 0; }
  template <typename ScenarioWrapper_T> requires ns::HasFixedPlanes<ScenarioWrapper_T>
  size_t getNumberOfFixedPlanes(ScenarioWrapper_T const& s)
  {
    return s.fixedPlanes().size();
  }

  template <typename ScenarioWrapper_T>
  std::optional<std::tuple<app::Point3, app::Vector3, app::ValueType>>
  getSphereData(ScenarioWrapper_T const& /*s*/, size_t /*obj_nr*/)
  {
    return {};
  }

  template <typename ScenarioWrapper_T>
  requires ns::HasSpheres<ScenarioWrapper_T>
    std::optional<std::tuple<app::Point3, app::Vector3, app::ValueType>>
    getSphereData(ScenarioWrapper_T const& s, size_t obj_nr)
  {
    const auto& sphere = s.spheres()[obj_nr];
    return {{sphere->point(), sphere->velocity(), sphere->radius()}};
  }

  template <typename ScenarioWrapper_T>
  std::optional<std::tuple<app::Point3, app::Vector3>>
  getFixedPlaneData(ScenarioWrapper_T const& /*s*/, size_t /*obj_nr*/)
  {
    return {};
  }

  template <typename ScenarioWrapper_T> requires ns::HasFixedPlanes<ScenarioWrapper_T>
  std::optional<std::tuple<app::Point3, app::Vector3>>
  getFixedPlaneData(ScenarioWrapper_T const& s, size_t obj_nr)
  {
    const auto& plane = s.fixedPlanes()[obj_nr];
    return {{plane->point(), plane->normal()}};
  }

}   // namespace app::detail


#endif   // DTE3607_SCENARIOWRAPPER_PRIVATE_H
