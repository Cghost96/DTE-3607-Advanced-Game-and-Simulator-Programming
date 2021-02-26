#ifndef DTE3607_SCENARIOWRAPPER_H
#define DTE3607_SCENARIOWRAPPER_H

#include "types.h"
#include "scenariowrapper_private.h"

#include <coldet/bits/rigidbodies.h>

// stl
#include <mutex>

namespace app {

  struct ScenarioWrapperBase {

    // Frame - velocity - radius
    using SphereData = std::tuple<Frame3H, Vector3, ValueType, bool>;

    // Frame - point - normal
    using PlaneData = std::tuple<Frame3H, Point3, Vector3>;

    // Frame - point - u axis - v axis - thickness
    using LimitedPlaneData = std::tuple<Frame3H, Point3, Vector3, Vector3, ValueType>;

    // Frame - control net
    using BezierSurfaceData = std::tuple<Frame3H, ControlNet>;

    virtual ~ScenarioWrapperBase() = default;


    /******************************
     * Backend access functionality
     */
    virtual void construct()                 = 0;
    virtual void solve(NanoSeconds timestep) = 0;
    /* Backend access functionality END
     *********************************/

    /***************************************
     * Frontend data retrievel functionality
     */
    enum class GeometryType : int { NA, Sphere, Plane, LimitedPlane, BezierSurface };



    virtual size_t numberOfSpheres() const        = 0;
    virtual size_t numberOfPlanes() const         = 0;
    virtual size_t numberOfLimitedPlanes() const  = 0;
    virtual size_t numberOfBezierSurfaces() const = 0;
    size_t         totalNumberOfObjects() const {
      return numberOfSpheres() + numberOfPlanes() + numberOfLimitedPlanes() + numberOfBezierSurfaces();
    }
    size_t spheresOffset() const { return 0; }
    size_t planesOffset() const { return spheresOffset() + numberOfSpheres(); }
    size_t limitedPlanesOffset() const { return planesOffset() + numberOfPlanes(); }
    size_t bezierSurfacesOffset() const { return limitedPlanesOffset() + numberOfLimitedPlanes(); }

    GeometryType geometryType(size_t combined_obj_nr) const {
      if (combined_obj_nr < planesOffset())
        return GeometryType::Sphere;
      else if (combined_obj_nr < limitedPlanesOffset())
        return GeometryType::Plane;
      else if (combined_obj_nr < bezierSurfacesOffset())
        return GeometryType::LimitedPlane;
      else if (combined_obj_nr < totalNumberOfObjects())
        return GeometryType::BezierSurface;
      else
        return GeometryType::NA;
    }

    virtual std::optional<SphereData>        sphereData(size_t obj_nr) const        = 0;
    virtual std::optional<PlaneData>         planeData(size_t obj_nr) const         = 0;
    virtual std::optional<LimitedPlaneData>  limitedPlaneData(size_t obj_nr) const  = 0;
    virtual std::optional<BezierSurfaceData> bezierSurfaceData(size_t obj_nr) const = 0;

    /* Frontend data retrievel functionality END
    ********************************************/
  };

  template <typename ConstructorLambda_T, typename SolverLambda_T,
            typename ScenarioPtr_T = decltype(ConstructorLambda_T()())>
  struct ScenarioWrapper : ScenarioWrapperBase {

    using SphereData        = ScenarioWrapperBase::SphereData;
    using PlaneData         = ScenarioWrapperBase::PlaneData;
    using LimitedPlaneData  = ScenarioWrapperBase::LimitedPlaneData;
    using BeizerSurfaceData = ScenarioWrapperBase::BezierSurfaceData;

    ScenarioWrapper(ConstructorLambda_T C, SolverLambda_T S) : m_C{C}, m_S{S} {}
    ConstructorLambda_T m_C;
    SolverLambda_T      m_S;
    ScenarioPtr_T       m_scenario_ptr;
    mutable std::mutex  m_mutex;

    void construct() final;
    void solve(NanoSeconds timestep) final;

    size_t numberOfSpheres() const final;
    size_t numberOfPlanes() const final;
    size_t numberOfLimitedPlanes() const final;
    size_t numberOfBezierSurfaces() const final;

    std::optional<SphereData>        sphereData(size_t obj_nr) const final;
    std::optional<PlaneData>         planeData(size_t obj_nr) const final;
    std::optional<LimitedPlaneData>  limitedPlaneData(size_t obj_nr) const final;
    std::optional<BezierSurfaceData> bezierSurfaceData(size_t obj_nr) const final;
  };



  template <typename ConstructorLambda_T, typename SolverLambda_T, typename ScenarioPtr_T>
  void ScenarioWrapper<ConstructorLambda_T, SolverLambda_T, ScenarioPtr_T>::construct() {
    std::scoped_lock lock(m_mutex);
    m_scenario_ptr = m_C();
  }

  template <typename ConstructorLambda_T, typename SolverLambda_T, typename ScenarioPtr_T>
  void ScenarioWrapper<ConstructorLambda_T, SolverLambda_T, ScenarioPtr_T>::solve(NanoSeconds timestep) {
    std::scoped_lock lock(m_mutex);
    if (not m_scenario_ptr) return;

    m_S(*m_scenario_ptr, timestep);
  }

  template <typename ConstructorLambda_T, typename SolverLambda_T, typename ScenarioPtr_T>
  size_t ScenarioWrapper<ConstructorLambda_T, SolverLambda_T, ScenarioPtr_T>::numberOfSpheres() const {
    std::scoped_lock lock(m_mutex);
    if (not m_scenario_ptr) return {};

    return detail::getNumberOfSpheres(*m_scenario_ptr);
  }

  template <typename ConstructorLambda_T, typename SolverLambda_T, typename ScenarioPtr_T>
  size_t ScenarioWrapper<ConstructorLambda_T, SolverLambda_T, ScenarioPtr_T>::numberOfPlanes() const {
    std::scoped_lock lock(m_mutex);
    if (not m_scenario_ptr) return {};

    return detail::getNumberOfFixedPlanes(*m_scenario_ptr);
  }

  template <typename ConstructorLambda_T, typename SolverLambda_T, typename ScenarioPtr_T>
  size_t ScenarioWrapper<ConstructorLambda_T, SolverLambda_T, ScenarioPtr_T>::numberOfLimitedPlanes() const {
    std::scoped_lock lock(m_mutex);
    if (not m_scenario_ptr) return {};

    return detail::getNumberOfFixedLimitedPlanes(*m_scenario_ptr);
  }


  template <typename ConstructorLambda_T, typename SolverLambda_T, typename ScenarioPtr_T>
  size_t ScenarioWrapper<ConstructorLambda_T, SolverLambda_T, ScenarioPtr_T>::numberOfBezierSurfaces() const {
    std::scoped_lock lock(m_mutex);
    if (not m_scenario_ptr) return {};

    return detail::getNumberOfFixedBezierSurfaces(*m_scenario_ptr);
  }

  template <typename ConstructorLambda_T, typename SolverLambda_T, typename ScenarioPtr_T>
  std::optional<ScenarioWrapperBase::SphereData>
  ScenarioWrapper<ConstructorLambda_T, SolverLambda_T, ScenarioPtr_T>::sphereData(size_t obj_nr) const {
    std::scoped_lock lock(m_mutex);
    if (not m_scenario_ptr) return {};

    return detail::getSphereData<ScenarioWrapperBase::SphereData>(*m_scenario_ptr, obj_nr);
  }

  template <typename ConstructorLambda_T, typename SolverLambda_T, typename ScenarioPtr_T>
  std::optional<ScenarioWrapperBase::PlaneData>
  ScenarioWrapper<ConstructorLambda_T, SolverLambda_T, ScenarioPtr_T>::planeData(size_t obj_nr) const {
    std::scoped_lock lock(m_mutex);
    if (not m_scenario_ptr) return {};

    return detail::getFixedPlaneData<ScenarioWrapperBase::PlaneData>(*m_scenario_ptr, obj_nr);
  }

  template <typename ConstructorLambda_T, typename SolverLambda_T, typename ScenarioPtr_T>
  std::optional<ScenarioWrapperBase::LimitedPlaneData>
  ScenarioWrapper<ConstructorLambda_T, SolverLambda_T, ScenarioPtr_T>::limitedPlaneData(size_t obj_nr) const {
    std::scoped_lock lock(m_mutex);
    if (not m_scenario_ptr) return {};

    return detail::getFixedLimitedPlaneData<ScenarioWrapperBase::LimitedPlaneData>(*m_scenario_ptr, obj_nr);
  }

  template <typename ConstructorLambda_T, typename SolverLambda_T, typename ScenarioPtr_T>
  std::optional<ScenarioWrapperBase::BezierSurfaceData>
  ScenarioWrapper<ConstructorLambda_T, SolverLambda_T, ScenarioPtr_T>::bezierSurfaceData(
    size_t obj_nr) const {
    std::scoped_lock lock(m_mutex);
    if (not m_scenario_ptr) return {};

    return detail::getFixedBezierSurfaceData<ScenarioWrapperBase::BezierSurfaceData>(*m_scenario_ptr, obj_nr);
  }

}   // namespace app

#endif   // DTE3607_SCENARIOWRAPPER_H
