#ifndef GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_SPHEREGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_SPHEREGEOMETRY_H


#include "../../tensorproductsurfacegeometry.h"

// gmlib
#include "gmlib/parametric/classic_objects/sphere.h"


namespace gmqt
{

  class SphereGeometry : public TensorProductSurfaceGeometry {
    Q_OBJECT

    Q_PROPERTY(float radius READ radius WRITE setRadius NOTIFY radiusChanged)

    using SphereType
      = gm::parametric::Sphere<gm::spaces::ProjectiveSpace<double, 3ul>>;

  public:
    SphereGeometry(QQuick3DObject* parent = nullptr)
      : TensorProductSurfaceGeometry(parent)
    {
      m_sphere = std::make_unique<SphereType>();
    }

    float radius() const { return float(m_sphere->m_radius); }

    void setRadius(float radius)
    {
      m_sphere->m_radius = double(radius);
      emit radiusChanged(radius);
      updateAndMarkDirty();
    }

  signals:
    void radiusChanged( float radius );


  public:
    using TensorProductSurfaceGeometry::SurfaceType;
    SurfaceType* internalSurface() const override { return m_sphere.get(); }

  private:
    std::unique_ptr<SphereType> m_sphere;
  };

}   // namespace gmqt


#endif   // GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_SPHEREGEOMETRY_H
