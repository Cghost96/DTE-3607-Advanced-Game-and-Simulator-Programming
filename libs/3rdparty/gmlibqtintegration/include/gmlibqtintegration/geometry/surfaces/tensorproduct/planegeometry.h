#ifndef GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_PLANEGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_PLANEGEOMETRY_H


#include "../../tensorproductsurfacegeometry.h"

// gmlib
#include "gmlib/parametric/classic_objects/plane.h"


namespace gmqt
{

  class PlaneGeometry : public TensorProductSurfaceGeometry {
    Q_OBJECT

    Q_PROPERTY(
      QVector3D origin READ origin WRITE setOrigin NOTIFY originChanged)
    Q_PROPERTY(QVector3D uAxis READ uAxis WRITE setUAxis NOTIFY uAxisChanged)
    Q_PROPERTY(QVector3D vAxis READ vAxis WRITE setVAxis NOTIFY vAxisChanged)


    using PlaneType
      = gm::parametric::Plane<gm::spaces::ProjectiveSpace<double, 3ul>>;

  public:
    PlaneGeometry(QQuick3DObject* parent = nullptr)
      : TensorProductSurfaceGeometry(parent)
    {
      m_plane = std::make_unique<PlaneType>();
      updateAndMarkDirty();
    }

    QVector3D origin() const { return utils::toQVector3D(m_plane->m_pt); }
    QVector3D uAxis() const { return utils::toQVector3D(m_plane->m_u); }
    QVector3D vAxis() const { return utils::toQVector3D(m_plane->m_v); }

    void setOrigin(QVector3D const& origin)
    {
      m_plane->m_pt = utils::toGMPointH4D(origin);
      emit originChanged(origin);
      updateAndMarkDirty();
    }

    void setUAxis(QVector3D const& uaxis)
    {
      m_plane->m_u = utils::toGMVectorH4D(uaxis);
      emit uAxisChanged(uaxis);
      updateAndMarkDirty();
    }

    void setVAxis(QVector3D const& vaxis)
    {
      m_plane->m_v = utils::toGMVectorH4D(vaxis);
      emit vAxisChanged(vaxis);
      updateAndMarkDirty();
    }

    using TensorProductSurfaceGeometry::SurfaceType;
    SurfaceType* internalSurface() const override { return m_plane.get(); }

  signals:
    void originChanged( QVector3D origin );
    void uAxisChanged( QVector3D uAxis );
    void vAxisChanged( QVector3D vAxis );


  private:
    std::unique_ptr<PlaneType> m_plane;
  };

}   // namespace gmqt


#endif   // GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_PLANEGEOMETRY_H
