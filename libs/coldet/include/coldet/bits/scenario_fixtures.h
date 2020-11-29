#ifndef DTE3607_COLDET_BITS_SCENARIO_FIXTURES_H
#define DTE3607_COLDET_BITS_SCENARIO_FIXTURES_H


#include "rigidbodies.h"
#include "../utils/to_string.h"

namespace dte3607::coldet::scenario_fixtures
{

  namespace detail
  {
    struct Forces {
      types::Vector3 G;
    };
  };   // namespace detail


  /***************************
   * Fixture Naming Convention
   *
   * Fixture
   *   O<non_constrained_objects_types>
   *   [C<constrained_objects_types>]
   *   [F<fixed_objects_types>]
   *   [P<properties>] ?? caching or other stuff -- feel free
   */

  struct FixtureOs {
    using Forces    = detail::Forces;
    using Sphere    = rigidbodies::Sphere;
    using SpherePtr = std::unique_ptr<Sphere>;
    using Spheres   = std::vector<SpherePtr>;

    Spheres const& spheres() const { return m_spheres; }
    Spheres&       spheres() { return m_spheres; }
    Forces const&  forces() const { return m_forces; }
    Forces&        forces() { return m_forces; }

  private:
    Spheres m_spheres;
    Forces  m_forces;
  };


  struct FixtureOsFp {
    using Forces    = detail::Forces;
    using Sphere    = rigidbodies::Sphere;
    using SpherePtr = std::unique_ptr<Sphere>;
    using Spheres   = std::vector<SpherePtr>;

    using FixedPlane    = rigidbodies::FixedPlane;
    using FixedPlanePtr = std::unique_ptr<FixedPlane>;
    using FixedPlanes   = std::vector<FixedPlanePtr>;

    Spheres const&     spheres() const { return m_spheres; }
    Spheres&           spheres() { return m_spheres; }
    Forces const&      forces() const { return m_forces; }
    Forces&            forces() { return m_forces; }
    FixedPlanes const& fixedPlanes() const { return m_fixed_planes; }
    FixedPlanes&       fixedPlanes() { return m_fixed_planes; }

  private:
    std::vector<SpherePtr>     m_spheres;
    std::vector<FixedPlanePtr> m_fixed_planes;
    Forces                     m_forces;
  };


}   // namespace dte3607::coldet::scenario_fixtures



#endif // DTE3607_COLDET_BITS_SCENARIO_FIXTURES_H
