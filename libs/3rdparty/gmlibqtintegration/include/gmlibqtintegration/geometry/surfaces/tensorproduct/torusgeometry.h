#ifndef GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_TORUSGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_TORUSGEOMETRY_H


#include "../../tensorproductsurfacegeometry.h"

// gmlib
#include "gmlib/parametric/classic_objects/torus.h"


namespace gmqt
{

  class TorusGeometry : public TensorProductSurfaceGeometry {
    Q_OBJECT

    Q_PROPERTY(float wheelRadius READ wheelRadius WRITE setWheelRadius NOTIFY
               wheelRadiusChanged)
    Q_PROPERTY(float tubeRadius1 READ tubeRadius1 WRITE setTubeRadius1 NOTIFY
               tubeRadius1Changed)
    Q_PROPERTY(float tubeRadius2 READ tubeRadius2 WRITE setTubeRadius2 NOTIFY
               tubeRadius2Changed)


    using TorusType
      = gm::parametric::Torus<gm::spaces::ProjectiveSpace<double, 3ul>>;

  public:
    TorusGeometry(QQuick3DObject* parent = nullptr)
      : TensorProductSurfaceGeometry(parent)
    {
      m_torus = std::make_unique<TorusType>();
    }

    float wheelRadius() const { return float(m_torus->m_wheelradius); }
    float tubeRadius1() const { return float(m_torus->m_tuberadius1); }
    float tubeRadius2() const { return float(m_torus->m_tuberadius2); }

    void setWheelRadius(float wheelradius) {
      m_torus->m_wheelradius = double(wheelradius);
      emit wheelRadiusChanged(wheelradius);
      updateAndMarkDirty();
    }
    void setTubeRadius1(float tuberadius1) {
      m_torus->m_tuberadius1 = double(tuberadius1);
      emit tubeRadius1Changed(tuberadius1);
      updateAndMarkDirty();
    }
    void setTubeRadius2(float tuberadius2) {
      m_torus->m_tuberadius2 = double(tuberadius2);
      emit tubeRadius2Changed(tuberadius2);
      updateAndMarkDirty();
    }

  signals:
    void wheelRadiusChanged( float wheelRadius );
    void tubeRadius1Changed( float tubeRadius1 );
    void tubeRadius2Changed( float tubeRadius2 );


  public:
    using TensorProductSurfaceGeometry::SurfaceType;
    SurfaceType* internalSurface() const override { return m_torus.get(); }

  private:
    std::unique_ptr<TorusType> m_torus;
  };

}   // namespace gmqt


#endif   // GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_TORUSGEOMETRY_H
