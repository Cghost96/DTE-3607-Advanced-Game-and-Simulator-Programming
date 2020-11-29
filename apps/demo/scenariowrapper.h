#ifndef DTE3607_SCENARIOWRAPPER_H
#define DTE3607_SCENARIOWRAPPER_H

#include "types.h"
#include "scenariowrapper_private.h"

// stl
#include <mutex>

namespace app
{

  struct ScenarioWrapperBase {
    virtual ~ScenarioWrapperBase()             = default;
    virtual void   construct()                 = 0;
    virtual void   solve(NanoSeconds timestep) = 0;
    virtual size_t numberOfSpheres() const     = 0;
    virtual size_t numberOfPlanes() const      = 0;

    // global position, global velocity, radius
    virtual std::optional<std::tuple<Point3, Vector3, ValueType>>
    sphereData(size_t obj_nr) const = 0;

    // global position, global normal
    virtual std::optional<std::tuple<Point3, Vector3>>
    fixedPlaneData(size_t obj_nr) const = 0;
  };

  template <typename ConstructorLambda_T, typename SolverLambda_T,
            typename ScenarioPtr_T = decltype(ConstructorLambda_T()())>
  struct ScenarioWrapper : ScenarioWrapperBase {

    ScenarioWrapper(ConstructorLambda_T C, SolverLambda_T S) : m_C{C}, m_S{S} {}
    ConstructorLambda_T m_C;
    SolverLambda_T      m_S;
    ScenarioPtr_T       m_scenario_ptr;
    mutable std::mutex  m_mutex;

    void construct() override;
    void solve(NanoSeconds timestep) override;


    size_t numberOfSpheres() const override;
    size_t numberOfPlanes() const override;

    std::optional<std::tuple<Point3, Vector3, ValueType>>
    sphereData(size_t obj_nr) const override;

    virtual std::optional<std::tuple<Point3, Vector3>>
    fixedPlaneData(size_t obj_nr) const override;
  };



  template <typename ConstructorLambda_T, typename SolverLambda_T,
            typename ScenarioPtr_T>
  void ScenarioWrapper<ConstructorLambda_T, SolverLambda_T,
                       ScenarioPtr_T>::construct()
  {
    std::scoped_lock lock(m_mutex);
    m_scenario_ptr = m_C();
  }

  template <typename ConstructorLambda_T, typename SolverLambda_T,
            typename ScenarioPtr_T>
  void
  ScenarioWrapper<ConstructorLambda_T, SolverLambda_T, ScenarioPtr_T>::solve(
    NanoSeconds timestep)
  {
    std::scoped_lock lock(m_mutex);
    if (not m_scenario_ptr) return;

    m_S(*m_scenario_ptr, timestep);
  }

  template <typename ConstructorLambda_T, typename SolverLambda_T,
            typename ScenarioPtr_T>
  size_t ScenarioWrapper<ConstructorLambda_T, SolverLambda_T,
                         ScenarioPtr_T>::numberOfSpheres() const
  {
    std::scoped_lock lock(m_mutex);
    if (not m_scenario_ptr) return {};

    return detail::getNumberOfSpheres(*m_scenario_ptr);
  }

  template <typename ConstructorLambda_T, typename SolverLambda_T,
            typename ScenarioPtr_T>
  size_t ScenarioWrapper<ConstructorLambda_T, SolverLambda_T,
                         ScenarioPtr_T>::numberOfPlanes() const
  {
    std::scoped_lock lock(m_mutex);
    if (not m_scenario_ptr) return {};

    return detail::getNumberOfFixedPlanes(*m_scenario_ptr);
  }

  template <typename ConstructorLambda_T, typename SolverLambda_T,
            typename ScenarioPtr_T>
  std::optional<std::tuple<Point3, Vector3, ValueType>>
  ScenarioWrapper<ConstructorLambda_T, SolverLambda_T,
                  ScenarioPtr_T>::sphereData(size_t obj_nr) const
  {
    std::scoped_lock lock(m_mutex);
    if (not m_scenario_ptr) return {};

    return detail::getSphereData(*m_scenario_ptr, obj_nr);
  }

  template <typename ConstructorLambda_T, typename SolverLambda_T,
            typename ScenarioPtr_T>
  std::optional<std::tuple<Point3, Vector3>>
  ScenarioWrapper<ConstructorLambda_T, SolverLambda_T,
                  ScenarioPtr_T>::fixedPlaneData(size_t obj_nr) const
  {
    std::scoped_lock lock(m_mutex);
    if (not m_scenario_ptr) return {};

    return detail::getFixedPlaneData(*m_scenario_ptr, obj_nr);
  }

}   // namespace app

#endif   // DTE3607_SCENARIOWRAPPER_H
