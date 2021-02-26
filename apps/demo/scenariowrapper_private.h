#ifndef DTE3607_SCENARIOWRAPPER_PRIVATE_H
#define DTE3607_SCENARIOWRAPPER_PRIVATE_H


#include "types.h"

// coldet
#include <coldet/concepts/scenario_fixture_concepts.h>

// stl
#include <optional>
#include <tuple>
#include <numbers>

namespace app::detail {

  namespace ns = dte3607::coldet::concepts::scenario_fixtures::solver_dev::components;

  template <typename Fixture_T>
  size_t getNumberOfSpheres(Fixture_T const& /*s*/) {
    return 0;
  }

  template <typename Fixture_T>
  requires ns::HasAllSpheres<Fixture_T> size_t getNumberOfSpheres(Fixture_T const& s) {
    return s.spheres().size() + s.fixedSpheres().size();
  }

  template <typename Fixture_T>
  size_t getNumberOfFixedPlanes(Fixture_T const& /*s*/) {
    return 0;
  }
  template <typename Fixture_T>
  requires ns::HasFixedPlanes<Fixture_T> size_t getNumberOfFixedPlanes(Fixture_T const& s) {
    return s.fixedPlanes().size();
  }

  template <typename Fixture_T>
  size_t getNumberOfFixedLimitedPlanes(Fixture_T const& /*s*/) {
    return 0;
  }
  template <typename Fixture_T>
  requires ns::HasFixedLimitedPlanes<Fixture_T> size_t getNumberOfFixedLimitedPlanes(Fixture_T const& s) {
    return s.fixedLimitedPlanes().size();
  }

  template <typename Fixture_T>
  size_t getNumberOfFixedBezierSurfaces(Fixture_T const& /*s*/) {
    return 0;
  }
  template <typename Fixture_T>
  requires ns::HasFixedBezierSurfaces<Fixture_T> size_t getNumberOfFixedBezierSurfaces(Fixture_T const& s) {
    return s.fixedBezierSurfaces().size();
  }

  template <typename SphereData_T, typename Fixture_T>
  std::optional<SphereData_T> getSphereData(Fixture_T const& /*s*/, size_t /*obj_nr*/) {
    return {};
  }

  template <typename SphereData_T, typename Fixture_T>
  requires ns::HasSpheres<Fixture_T> std::optional<SphereData_T> getSphereData(Fixture_T const& s,
                                                                               size_t           obj_nr) {
    const auto& sphere = s.spheres()[obj_nr];

    return {{sphere->spaceObjectFrame().pSpaceFrameParent(), sphere->velocity(), sphere->radius(), false}};
  }

  template <typename SphereData_T, typename Fixture_T>
  requires ns::HasAllSpheres<Fixture_T> std::optional<SphereData_T> getSphereData(Fixture_T const& s,
                                                                                  size_t           obj_nr) {
    bool fs = obj_nr >= s.spheres().size();
    if (fs) {
      obj_nr -= s.spheres().size();
      const auto& fixed_sphere = s.fixedSpheres()[obj_nr];
      return {{fixed_sphere->spaceObjectFrame().pSpaceFrameParent(), fixed_sphere->velocity(),
               fixed_sphere->radius(), fs}};
    }
    else {
      const auto& sphere = s.spheres()[obj_nr];

      return {{sphere->spaceObjectFrame().pSpaceFrameParent(), sphere->velocity(), sphere->radius(), fs}};
    }
  }

  template <typename FixedPlaneData_T, typename Fixture_T>
  std::optional<FixedPlaneData_T> getFixedPlaneData(Fixture_T const& /*s*/, size_t /*obj_nr*/) {
    return {};
  }

  template <typename FixedPlaneData_T, typename Fixture_T>
  requires ns::HasFixedPlanes<Fixture_T> std::optional<FixedPlaneData_T> getFixedPlaneData(Fixture_T const& s,
                                                                                           size_t obj_nr) {
    const auto& plane = s.fixedPlanes()[obj_nr];
    return {{plane->spaceObjectFrame().pSpaceFrameParent(), plane->point(), plane->normal()}};
  }

  template <typename FixedLimitedPlaneData_T, typename Fixture_T>
  std::optional<FixedLimitedPlaneData_T> getFixedLimitedPlaneData(Fixture_T const& /*s*/, size_t /*obj_nr*/) {
    return {};
  }

  template <typename FixedLimitedPlaneData_T, typename Fixture_T>
  requires ns::HasFixedLimitedPlanes<Fixture_T> std::optional<FixedLimitedPlaneData_T>
                                                getFixedLimitedPlaneData(Fixture_T const& s, size_t obj_nr) {
    const auto& plane = s.fixedLimitedPlanes()[obj_nr];
    return {{plane->spaceObjectFrame().pSpaceFrameParent(), plane->pointLocal(), plane->uAxisLocal(),
             plane->vAxisLocal(), plane->thickness()}};
  }

  template <typename FixedBezierSurfaceData_T, typename Fixture_T>
  std::optional<FixedBezierSurfaceData_T> getFixedBezierSurfaceData(Fixture_T const& /*s*/,
                                                                    size_t /*obj_nr*/) {
    return {};
  }

  template <typename FixedBezierSurfaceData_T, typename Fixture_T>
  requires ns::HasFixedBezierSurfaces<Fixture_T> std::optional<FixedBezierSurfaceData_T>
  getFixedBezierSurfaceData(Fixture_T const& s, size_t obj_nr) {
    const auto& bez = s.fixedBezierSurfaces()[obj_nr];
    return {{bez->spaceObjectFrame().pSpaceFrameParent(), bez->bezierSurface().m_C}};
  }

}   // namespace app::detail


#endif   // DTE3607_SCENARIOWRAPPER_PRIVATE_H
