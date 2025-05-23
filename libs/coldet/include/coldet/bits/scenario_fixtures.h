#ifndef DTE3607_COLDET_BITS_SCENARIO_FIXTURES_H
#define DTE3607_COLDET_BITS_SCENARIO_FIXTURES_H


#include "rigidbodies.h"
#include "../utils/to_string.h"

#include <unordered_map>
#include <vector>

namespace dte3607::coldet::scenario_fixtures {

  namespace detail {
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

  struct FixtureOsFlp {
    using Forces    = detail::Forces;
    using Sphere    = rigidbodies::Sphere;
    using SpherePtr = std::unique_ptr<Sphere>;
    using Spheres   = std::vector<SpherePtr>;

    using FixedLimitedPlane    = rigidbodies::FixedLimitedPlane;
    using FixedLimitedPlanePtr = std::unique_ptr<FixedLimitedPlane>;
    using FixedLimitedPlanes   = std::vector<FixedLimitedPlanePtr>;

    Forces const& forces() const { return m_forces; }
    Forces&       forces() { return m_forces; }

    Spheres const& spheres() const { return m_spheres; }
    Spheres&       spheres() { return m_spheres; }

    FixedLimitedPlanes const& fixedLimitedPlanes() const { return m_fixed_limited_planes; }
    FixedLimitedPlanes&       fixedLimitedPlanes() { return m_fixed_limited_planes; }

  private:
    Spheres            m_spheres;
    FixedLimitedPlanes m_fixed_limited_planes;
    Forces             m_forces;
  };

  struct FixtureOsFb {
    using Forces = detail::Forces;

    using Sphere    = rigidbodies::Sphere;
    using SpherePtr = std::unique_ptr<Sphere>;
    using Spheres   = std::vector<SpherePtr>;

    using FixedBezierSurface    = rigidbodies::FixedBezierSurface;
    using FixedBezierSurfacePtr = std::unique_ptr<FixedBezierSurface>;
    using FixedBezierSurfaces   = std::vector<FixedBezierSurfacePtr>;

    Forces const& forces() const { return m_forces; }
    Forces&       forces() { return m_forces; }

    Spheres const& spheres() const { return m_spheres; }
    Spheres&       spheres() { return m_spheres; }

    FixedBezierSurfaces const& fixedBezierSurfaces() const { return m_fixed_bezsurfaces; }
    FixedBezierSurfaces&       fixedBezierSurfaces() { return m_fixed_bezsurfaces; }

  private:
    Spheres             m_spheres;
    FixedBezierSurfaces m_fixed_bezsurfaces;
    Forces              m_forces;
  };

  // Sphere to fixed plane single attachment
  struct FixtureOsFpPsfp_sa {
    using Forces = detail::Forces;

    using Sphere    = rigidbodies::Sphere;
    using SpherePtr = std::unique_ptr<Sphere>;
    using Spheres   = std::vector<SpherePtr>;

    using FixedPlane    = rigidbodies::FixedPlane;
    using FixedPlanePtr = std::unique_ptr<FixedPlane>;
    using FixedPlanes   = std::vector<FixedPlanePtr>;

    using SFPAttachments = std::unordered_map<Sphere*, FixedPlane*>;

    Forces const&         forces() const { return m_forces; }
    Forces&               forces() { return m_forces; }
    Spheres const&        spheres() const { return m_spheres; }
    Spheres&              spheres() { return m_spheres; }
    FixedPlanes const&    fixedPlanes() const { return m_fixed_planes; }
    FixedPlanes&          fixedPlanes() { return m_fixed_planes; }
    SFPAttachments const& sfpAttachments() const { return m_attachments; }
    SFPAttachments&       sfpAttachments() { return m_attachments; }

  private:
    Forces                                   m_forces;
    std::vector<SpherePtr>                   m_spheres;
    std::vector<FixedPlanePtr>               m_fixed_planes;
    std::unordered_map<Sphere*, FixedPlane*> m_attachments;
  };

  // Sphere to fixed plane single attachment and ds cache
  struct FixtureOsFpPsfp_sa_ds {
    using Forces = detail::Forces;

    using Sphere    = rigidbodies::Sphere;
    using SpherePtr = std::unique_ptr<Sphere>;
    using Spheres   = std::vector<SpherePtr>;

    using FixedPlane    = rigidbodies::FixedPlane;
    using FixedPlanePtr = std::unique_ptr<FixedPlane>;
    using FixedPlanes   = std::vector<FixedPlanePtr>;

    using SFPAttachments = std::unordered_map<Sphere*, FixedPlane*>;
    using V3             = types::Vector3;
    using Trajectories   = std::unordered_map<Sphere*, std::pair<V3, V3>>;

    Forces const&         forces() const { return m_forces; }
    Forces&               forces() { return m_forces; }
    Spheres const&        spheres() const { return m_spheres; }
    Spheres&              spheres() { return m_spheres; }
    FixedPlanes const&    fixedPlanes() const { return m_fixed_planes; }
    FixedPlanes&          fixedPlanes() { return m_fixed_planes; }
    SFPAttachments const& sfpAttachments() const { return m_attachments; }
    SFPAttachments&       sfpAttachments() { return m_attachments; }
    Trajectories const&   trajectories() const { return m_trajectories; }
    Trajectories&         trajectories() { return m_trajectories; }

  private:
    Forces                                         m_forces;
    std::vector<SpherePtr>                         m_spheres;
    std::vector<FixedPlanePtr>                     m_fixed_planes;
    std::unordered_map<Sphere*, FixedPlane*>       m_attachments;
    std::unordered_map<Sphere*, std::pair<V3, V3>> m_trajectories;
  };

  // Dynamic sphere, fixed sphere, fixed infinite plane, fixed limited plane, fixed cylinder, sphere-fixed
  // plane single attachment, ds cache
  struct Fixture_Os_Fs_ip_lp_cPsfp_sa_ds {
    using Forces = detail::Forces;
    using V3     = types::Vector3;

    using Sphere    = rigidbodies::Sphere;
    using SpherePtr = std::unique_ptr<Sphere>;
    using Spheres   = std::vector<SpherePtr>;

    using FixedSphere    = rigidbodies::FixedSphere;
    using FixedSpherePtr = std::unique_ptr<FixedSphere>;
    using FixedSpheres   = std::vector<FixedSpherePtr>;

    using FixedPlane    = rigidbodies::FixedPlane;
    using FixedPlanePtr = std::unique_ptr<FixedPlane>;
    using FixedPlanes   = std::vector<FixedPlanePtr>;

    using FixedLimitedPlane    = rigidbodies::FixedLimitedPlane;
    using FixedLimitedPlanePtr = std::unique_ptr<FixedLimitedPlane>;
    using FixedLimitedPlanes   = std::vector<FixedLimitedPlanePtr>;

    using OsFpAttachments = std::unordered_map<Sphere*, FixedPlane*>;
    using Trajectories    = std::unordered_map<Sphere*, std::pair<V3, V3>>;

    Forces const& forces() const { return m_forces; }
    Forces&       forces() { return m_forces; }

    Spheres const& spheres() const { return m_spheres; }
    Spheres&       spheres() { return m_spheres; }

    FixedSpheres const& fixedSpheres() const { return m_fixed_spheres; }
    FixedSpheres&       fixedSpheres() { return m_fixed_spheres; }

    FixedPlanes const& fixedPlanes() const { return m_fixed_planes; }
    FixedPlanes&       fixedPlanes() { return m_fixed_planes; }

    FixedLimitedPlanes const& fixedLimitedPlanes() const { return m_fixed_limited_planes; }
    FixedLimitedPlanes&       fixedLimitedPlanes() { return m_fixed_limited_planes; }


    OsFpAttachments const& attachmentsOsFp() const { return m_OsFp_attachments; }
    OsFpAttachments&       attachmentsOsFp() { return m_OsFp_attachments; }

    Trajectories const& trajectories() const { return m_trajectories; }
    Trajectories&       trajectories() { return m_trajectories; }

    double const& pyrTopPoint() const { return m_pyr_top_point; }
    double&       pyrTopPoint() { return m_pyr_top_point; }

  private:
    Forces             m_forces;
    Spheres            m_spheres;
    FixedSpheres       m_fixed_spheres;
    FixedPlanes        m_fixed_planes;
    FixedLimitedPlanes m_fixed_limited_planes;
    OsFpAttachments    m_OsFp_attachments;
    Trajectories       m_trajectories;
    double             m_pyr_top_point;
  };

  // Dynamic sphere, fixed sphere, fixed infinite plane, fixed limited plane, fixed cylinder, sphere-fixed
  // plane single attachment, ds cache, timer for sphere handling
  struct Fixture_Os_Fs_ip_lp_cPsfp_sa_ds_t {
    using Forces = detail::Forces;
    using V3     = types::Vector3;
    using NS     = types::NanoSeconds;

    using Sphere    = rigidbodies::Sphere;
    using SpherePtr = std::unique_ptr<Sphere>;
    using Spheres   = std::vector<SpherePtr>;

    using FixedSphere    = rigidbodies::FixedSphere;
    using FixedSpherePtr = std::unique_ptr<FixedSphere>;
    using FixedSpheres   = std::vector<FixedSpherePtr>;

    using FixedPlane    = rigidbodies::FixedPlane;
    using FixedPlanePtr = std::unique_ptr<FixedPlane>;
    using FixedPlanes   = std::vector<FixedPlanePtr>;

    using FixedLimitedPlane    = rigidbodies::FixedLimitedPlane;
    using FixedLimitedPlanePtr = std::unique_ptr<FixedLimitedPlane>;
    using FixedLimitedPlanes   = std::vector<FixedLimitedPlanePtr>;

    using OsFpAttachments = std::unordered_map<Sphere*, FixedPlane*>;
    using Trajectories    = std::unordered_map<Sphere*, std::pair<V3, V3>>;

    Forces const& forces() const { return m_forces; }
    Forces&       forces() { return m_forces; }

    Spheres const& spheres() const { return m_spheres; }
    Spheres&       spheres() { return m_spheres; }

    FixedSpheres const& fixedSpheres() const { return m_fixed_spheres; }
    FixedSpheres&       fixedSpheres() { return m_fixed_spheres; }

    FixedPlanes const& fixedPlanes() const { return m_fixed_planes; }
    FixedPlanes&       fixedPlanes() { return m_fixed_planes; }

    FixedLimitedPlanes const& fixedLimitedPlanes() const { return m_fixed_limited_planes; }
    FixedLimitedPlanes&       fixedLimitedPlanes() { return m_fixed_limited_planes; }

    OsFpAttachments const& attachmentsOsFp() const { return m_OsFp_attachments; }
    OsFpAttachments&       attachmentsOsFp() { return m_OsFp_attachments; }

    Trajectories const& trajectories() const { return m_trajectories; }
    Trajectories&       trajectories() { return m_trajectories; }

    double const& pyrTopPoint() const { return m_pyr_top_point; }
    double&       pyrTopPoint() { return m_pyr_top_point; }

    double const& time() const { return m_time; }
    double&       time() { return m_time; }

    int const& sIndex() const { return m_s_index; }
    int&       sIndex() { return m_s_index; }

  private:
    Forces             m_forces;
    Spheres            m_spheres;
    FixedSpheres       m_fixed_spheres;
    FixedPlanes        m_fixed_planes;
    FixedLimitedPlanes m_fixed_limited_planes;
    OsFpAttachments    m_OsFp_attachments;
    Trajectories       m_trajectories;
    double             m_pyr_top_point;
    double             m_time;
    int                m_s_index;
  };

  // Dynamic sphere, fixed sphere, fixed infinite plane, fixed limited plane, fixed cylinder, sphere-fixed
  // plane single attachment, ds cache, counter for data generation, vector for sphere distribution
  struct Fixture_Os_Fs_ip_lp_cPsfp_sa_ds_counter_distribution {
    using Forces = detail::Forces;
    using V3     = types::Vector3;

    using Sphere    = rigidbodies::Sphere;
    using SpherePtr = std::unique_ptr<Sphere>;
    using Spheres   = std::vector<SpherePtr>;

    using FixedSphere    = rigidbodies::FixedSphere;
    using FixedSpherePtr = std::unique_ptr<FixedSphere>;
    using FixedSpheres   = std::vector<FixedSpherePtr>;

    using FixedPlane    = rigidbodies::FixedPlane;
    using FixedPlanePtr = std::unique_ptr<FixedPlane>;
    using FixedPlanes   = std::vector<FixedPlanePtr>;

    using FixedLimitedPlane    = rigidbodies::FixedLimitedPlane;
    using FixedLimitedPlanePtr = std::unique_ptr<FixedLimitedPlane>;
    using FixedLimitedPlanes   = std::vector<FixedLimitedPlanePtr>;

    using OsFpAttachments = std::unordered_map<Sphere*, FixedPlane*>;
    using Trajectories    = std::unordered_map<Sphere*, std::pair<V3, V3>>;

    using Distribution = std::vector<int>;

    Forces const& forces() const { return m_forces; }
    Forces&       forces() { return m_forces; }

    Spheres const& spheres() const { return m_spheres; }
    Spheres&       spheres() { return m_spheres; }

    FixedSpheres const& fixedSpheres() const { return m_fixed_spheres; }
    FixedSpheres&       fixedSpheres() { return m_fixed_spheres; }

    FixedPlanes const& fixedPlanes() const { return m_fixed_planes; }
    FixedPlanes&       fixedPlanes() { return m_fixed_planes; }

    FixedLimitedPlanes const& fixedLimitedPlanes() const { return m_fixed_limited_planes; }
    FixedLimitedPlanes&       fixedLimitedPlanes() { return m_fixed_limited_planes; }


    OsFpAttachments const& attachmentsOsFp() const { return m_OsFp_attachments; }
    OsFpAttachments&       attachmentsOsFp() { return m_OsFp_attachments; }

    Trajectories const& trajectories() const { return m_trajectories; }
    Trajectories&       trajectories() { return m_trajectories; }

    double const& pyrTopPoint() const { return m_pyr_top_point; }
    double&       pyrTopPoint() { return m_pyr_top_point; }

    int const& counter() const { return m_counter; }
    int&       counter() { return m_counter; }

    Distribution const& xdistribution() const { return m_x_distribution; }
    Distribution&       xdistribution() { return m_x_distribution; }

    Distribution const& zdistribution() const { return m_z_distribution; }
    Distribution&       zdistribution() { return m_z_distribution; }

  private:
    Forces             m_forces;
    Spheres            m_spheres;
    FixedSpheres       m_fixed_spheres;
    FixedPlanes        m_fixed_planes;
    FixedLimitedPlanes m_fixed_limited_planes;
    OsFpAttachments    m_OsFp_attachments;
    Trajectories       m_trajectories;
    double             m_pyr_top_point;
    int                m_counter;
    Distribution       m_x_distribution;
    Distribution       m_z_distribution;
  };

  // Dynamic sphere, fixed sphere, fixed infinite plane, fixed limited plane, fixed cylinder, sphere-fixed
  // plane single attachment, ds cache, timer for sphere handling, counter for data generation, vector for
  // sphere distribution
  struct Fixture_Os_Fs_ip_lp_cPsfp_sa_ds_t_counter_distribution {
    using Forces = detail::Forces;
    using V3     = types::Vector3;
    using NS     = types::NanoSeconds;

    using Sphere    = rigidbodies::Sphere;
    using SpherePtr = std::unique_ptr<Sphere>;
    using Spheres   = std::vector<SpherePtr>;

    using FixedSphere    = rigidbodies::FixedSphere;
    using FixedSpherePtr = std::unique_ptr<FixedSphere>;
    using FixedSpheres   = std::vector<FixedSpherePtr>;

    using FixedPlane    = rigidbodies::FixedPlane;
    using FixedPlanePtr = std::unique_ptr<FixedPlane>;
    using FixedPlanes   = std::vector<FixedPlanePtr>;

    using FixedLimitedPlane    = rigidbodies::FixedLimitedPlane;
    using FixedLimitedPlanePtr = std::unique_ptr<FixedLimitedPlane>;
    using FixedLimitedPlanes   = std::vector<FixedLimitedPlanePtr>;

    using OsFpAttachments = std::unordered_map<Sphere*, FixedPlane*>;
    using Trajectories    = std::unordered_map<Sphere*, std::pair<V3, V3>>;

    using Distribution = std::vector<int>;

    Forces const& forces() const { return m_forces; }
    Forces&       forces() { return m_forces; }

    Spheres const& spheres() const { return m_spheres; }
    Spheres&       spheres() { return m_spheres; }

    FixedSpheres const& fixedSpheres() const { return m_fixed_spheres; }
    FixedSpheres&       fixedSpheres() { return m_fixed_spheres; }

    FixedPlanes const& fixedPlanes() const { return m_fixed_planes; }
    FixedPlanes&       fixedPlanes() { return m_fixed_planes; }

    FixedLimitedPlanes const& fixedLimitedPlanes() const { return m_fixed_limited_planes; }
    FixedLimitedPlanes&       fixedLimitedPlanes() { return m_fixed_limited_planes; }

    OsFpAttachments const& attachmentsOsFp() const { return m_OsFp_attachments; }
    OsFpAttachments&       attachmentsOsFp() { return m_OsFp_attachments; }

    Trajectories const& trajectories() const { return m_trajectories; }
    Trajectories&       trajectories() { return m_trajectories; }

    double const& pyrTopPoint() const { return m_pyr_top_point; }
    double&       pyrTopPoint() { return m_pyr_top_point; }

    double const& time() const { return m_time; }
    double&       time() { return m_time; }

    int const& sIndex() const { return m_s_index; }
    int&       sIndex() { return m_s_index; }

    int const& counter() const { return m_counter; }
    int&       counter() { return m_counter; }

    Distribution const& xdistribution() const { return m_x_distribution; }
    Distribution&       xdistribution() { return m_x_distribution; }

    Distribution const& zdistribution() const { return m_z_distribution; }
    Distribution&       zdistribution() { return m_z_distribution; }

  private:
    Forces             m_forces;
    Spheres            m_spheres;
    FixedSpheres       m_fixed_spheres;
    FixedPlanes        m_fixed_planes;
    FixedLimitedPlanes m_fixed_limited_planes;
    OsFpAttachments    m_OsFp_attachments;
    Trajectories       m_trajectories;
    double             m_pyr_top_point;
    double             m_time;
    int                m_s_index;
    int                m_counter;
    Distribution       m_x_distribution;
    Distribution       m_z_distribution;
  };


}   // namespace dte3607::coldet::scenario_fixtures





#endif   // DTE3607_COLDET_BITS_SCENARIO_FIXTURES_H
