#ifndef DTE3607_COLDET_BITS_RIGIDBODIES_H
#define DTE3607_COLDET_BITS_RIGIDBODIES_H

#include "types.h"

// gmlib
#include <gmlib/parametric/classic_constructions/beziersurface.h>


// stl
#include <concepts>

namespace dte3607::coldet::rigidbodies {

  struct RigidBody {
    RigidBody(const std::string& name = "") : m_name{name} {}
    virtual ~RigidBody() = default;

    // Concept requirements
    using ValueType        = types::Point3::ElementType;
    using Point3           = types::Point3;
    using Point3H          = types::Point3H;
    using Vector3          = types::Vector3;
    using Vector3H         = types::Vector3H;
    using SpaceObjectFrame = types::ProjectiveSpaceObject;
    using Timepoint        = types::HighResolutionTP;

    // Class type constants
    static constexpr ValueType frictionCoefMin{0.0};
    static constexpr ValueType frictionCoefMax{1.0};

    // Lin. alg. prop. access
    virtual SpaceObjectFrame&       spaceObjectFrame() { return m_object; }
    virtual SpaceObjectFrame const& spaceObjectFrame() const { return m_object; }

    // Mechanics prop. access
    ValueType  mass() const { return m_mass; }
    ValueType  frictionCoef() const { return m_friction_coef; }
    Vector3    velocity() const { return m_velocity; }
    Timepoint& timepoint() { return m_timepoint; }
    void       setMass(ValueType const& mass) { m_mass = mass; }
    void       setFrictionCoef(ValueType const& friction_coef) {
      m_friction_coef = std::clamp(friction_coef, frictionCoefMin, frictionCoefMax);
    }

    // Other stuff
    /** a; in the "parent" spacial frame */
    void addAcceleration(types::Vector3 const& a) { m_velocity += a; }

    /** v; in the "parent" spacial frame */
    void setVelocity(types::Vector3 const& v) { m_velocity = v; }

    // States
    enum class States { Free, Resting, Sliding, Rolling };
    States& state() { return m_state; }

    void setState(States state) { m_state = States{state}; }

  private:
    SpaceObjectFrame m_object;
    Timepoint        m_timepoint;
    Vector3          m_velocity{0, 0, 0};
    ValueType        m_mass{1.0};
    ValueType        m_friction_coef{0.0};   // 0 == no friction
    States           m_state{States::Free};


    typename SpaceObjectFrame::Frame vFrame() const { return spaceObjectFrame().vSpaceFrameParent(); }

  public:
    std::string m_name;
  };



  //  template <std::floating_point ValueType_T = double>
  struct Sphere : RigidBody {
    Sphere(types::Vector3 const& velocity, types::Vector3::ElementType const& radius, std::string name = {})
      : RigidBody(name), m_radius{radius} {
      setVelocity(velocity);
    }

    // Sphere prop. access
    Point3    point() const { return spaceObjectFrame().frameOriginParent(); }
    ValueType radius() const { return m_radius; }

    // Concept requirements END

  private:
    ValueType m_radius{1.0};
  };


  struct FixedPlane : RigidBody {
    FixedPlane(types::Vector3 const& n, std::string name = {}) : RigidBody(name), m_n{n} {}

    // Plane prop. access
    Point3  point() const { return spaceObjectFrame().frameOriginParent(); }
    Vector3 normal() const { return spaceObjectFrame().vSpaceFrameParent() * m_n; }
    // Concept requirements END

  private:
    Vector3 m_n{0, 1, 0};
  };


  struct FixedLimitedPlane : RigidBody {
    FixedLimitedPlane(types::Point3 const& p, types::Vector3 const& u, types::Vector3 const& v,
                      types::ValueType const& thickness, std::string name = {})
      : RigidBody(name), m_u{u}, m_v{v}, m_thickness{thickness} {
      blaze::subvector<0ul, 3ul>(m_p) = p;
    }

    // Plane prop. access
    Point3 pointLocal() const { return blaze::subvector<0ul, 3ul>(m_p); }

    Vector3 uAxisLocal() const { return m_u; }
    Vector3 vAxisLocal() const { return m_v; }

    ValueType thickness() const { return m_thickness; }



    Point3 point() const { return blaze::subvector<0ul, 3ul>(spaceObjectFrame().pSpaceFrameParent() * m_p); }

    Vector3 normalFront() const { return spaceObjectFrame().vSpaceFrameParent() * blaze::cross(m_u, m_v); }

    Vector3 normalBack() const { return -normalFront(); }

    Vector3 partialDerivativeU() const { return spaceObjectFrame().vSpaceFrameParent() * m_u; }

    Vector3 partialDerivativeV() const { return spaceObjectFrame().vSpaceFrameParent() * m_v; }

    Point3 evaluate(ValueType u, ValueType v) const {
      return point() + u * partialDerivativeU() + v * partialDerivativeV();
    }

    Point3 evaluateFront(ValueType u, ValueType v) const {
      return evaluate(u, v) + m_thickness * normalFront();
    }
    Point3 evaluateBack(ValueType u, ValueType v) const {
      return evaluate(u, v) + m_thickness * normalBack();
    }

    // Concept requirements END

  private:
    Point3H   m_p{0, 0, 0, 1};
    Vector3   m_u{1, 0, 0};
    Vector3   m_v{0, 0, 1};
    ValueType m_thickness{0.1};
  };


  struct FixedBezierSurface : RigidBody {
    FixedBezierSurface(std::string const& name = {}) : RigidBody(name) {}

    using BezSurfType = gm::parametric::BezierSurface<SpaceObjectFrame::Space>;

    SpaceObjectFrame&       spaceObjectFrame() override { return m_bez_surf; }
    SpaceObjectFrame const& spaceObjectFrame() const override { return m_bez_surf; }

    BezSurfType&       bezierSurface() { return m_bez_surf; }
    BezSurfType const& bezierSurface() const { return m_bez_surf; }

  private:
    BezSurfType m_bez_surf;
  };

}   // namespace dte3607::coldet::rigidbodies



#endif   // DTE3607_COLDET_BITS_RIGIDBODIES_H
