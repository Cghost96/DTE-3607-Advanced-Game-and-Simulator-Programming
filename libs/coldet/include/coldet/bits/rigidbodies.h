#ifndef DTE3607_COLDET_BITS_RIGIDBODIES_H
#define DTE3607_COLDET_BITS_RIGIDBODIES_H

#include "types.h"


//stl
#include <concepts>

namespace dte3607::coldet::rigidbodies
{



//  template <std::floating_point ValueType_T = double>
  struct Sphere {
    Sphere(types::Vector3 const&              velocity,
           types::Vector3::ElementType const& radius, std::string name = {})
      : m_name{name}, m_velocity{velocity}, m_radius{radius}
    {
    }

    // Concept requirements
    using ValueType        = types::Point3::ElementType;
    using Point3           = types::Point3;
    using Point3H          = types::Point3H;
    using Vector3          = types::Vector3;
    using Vector3H         = types::Vector3H;
    using SpaceObjectFrame = types::ProjectiveSpaceObject;
    using Timepoint        = types::HighResolutionTP;

    // Lin. alg. prop. access
    SpaceObjectFrame&       spaceObjectFrame() { return m_object; }
    SpaceObjectFrame const& spaceObjectFrame() const { return m_object; }

    // Sphere prop. access
    Point3    point() const { return m_object.frameOriginParent(); }
    ValueType radius() const { return m_radius; }

    // Mechanics prop. access
    Vector3 velocity() const
    {
      return vFrame() * m_velocity;
    }
    Timepoint& timepoint() { return m_timepoint; }
    ValueType  mass() const { return m_mass; }
    ValueType  frictionCoef() const { return m_friction_coef; }

    // Other stuff
    /** a; in the "parent" spacial frame */
    void addAcceleration(types::Vector3 const& a) {
      m_velocity += blaze::inv(vFrame()) * a;
    }

    /** v; in the "parent" spacial frame */
    void setVelocity(types::Vector3 const& v)
    {
      m_velocity = blaze::inv(vFrame()) * v;
    }

    // States
    enum class States { Free, Resting, Sliding, Rolling };
    States& state() { return m_state; }


    // Concept requirements END





  public:
    std::string m_name;

  private:
    SpaceObjectFrame m_object;
    Timepoint        m_timepoint;
    Vector3          m_velocity{0, 0, 0};
    ValueType        m_radius{1.0};
    ValueType        m_mass{1.0};
    ValueType        m_friction_coef{0.0};
    States           m_state{States::Free};


    typename SpaceObjectFrame::Frame vFrame() const
    {
      return m_object.vSpaceFrameParent();
    }
  };


  struct FixedPlane {
    FixedPlane(types::Vector3 const& n, std::string name = {})
      : m_name{name}, m_n{n}
    {
    }

    // Concept requirements


    // Concept requirements
    using ValueType        = types::Point3::ElementType;
    using Point3           = types::Point3;
    using Point3H          = types::Point3H;
    using Vector3          = types::Vector3;
    using Vector3H         = types::Vector3H;
    using SpaceObjectFrame = types::ProjectiveSpaceObject;
    using Timepoint        = types::HighResolutionTP;

    // Lin. alg. prop. access
    SpaceObjectFrame& spaceObjectFrame() { return m_object; }

    // Plane prop. access
    Point3 point() const { return m_object.frameOriginParent(); }
    Point3 normal() const { return m_object.vSpaceFrameParent() * m_n; }

    ValueType  frictionCoef() const { return m_friction_coef; }

    // Concept requirements END

  public:
    std::string m_name;

  private:
    SpaceObjectFrame m_object;
    Vector3          m_n{0, 1, 0};
    ValueType        m_friction_coef{0.0};
  };

}   // namespace dte3607::coldet::rigidbodies



#endif // DTE3607_COLDET_BITS_RIGIDBODIES_H
