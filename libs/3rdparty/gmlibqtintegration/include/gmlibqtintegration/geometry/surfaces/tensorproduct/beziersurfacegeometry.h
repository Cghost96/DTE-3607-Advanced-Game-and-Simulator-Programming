#ifndef GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_BEZIERSURFACEGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_BEZIERSURFACEGEOMETRY_H


#include "../../tensorproductsurfacegeometry.h"

// gmlib
#include "gmlib/parametric/classic_constructions/beziersurface.h"


namespace gmqt
{

  class BezierSurfaceGeometry : public TensorProductSurfaceGeometry {
    Q_OBJECT

//    Q_PROPERTY(
//      QVector3D origin READ origin WRITE setOrigin NOTIFY originChanged)
//    Q_PROPERTY(QVector3D uAxis READ uAxis WRITE setUAxis NOTIFY uAxisChanged)
//    Q_PROPERTY(QVector3D vAxis READ vAxis WRITE setVAxis NOTIFY vAxisChanged)

  public:

    using BezierSurfaceType
      = gm::parametric::BezierSurface<gm::spaces::ProjectiveSpace<double, 3ul>>;

    BezierSurfaceGeometry(QQuick3DObject* parent = nullptr)
      : TensorProductSurfaceGeometry(parent)
    {
      m_bezsurf = std::make_unique<BezierSurfaceType>();
      updateAndMarkDirty();
    }

//    QVector3D origin() const { return utils::toQVector3D(m_plane->m_pt); }
//    QVector3D uAxis() const { return utils::toQVector3D(m_plane->m_u); }
//    QVector3D vAxis() const { return utils::toQVector3D(m_plane->m_v); }

//    void setOrigin(QVector3D const& origin)
//    {
//      m_plane->m_pt = utils::toGMPointH4D(origin);
//      emit originChanged(origin);
//      updateAndMarkDirty();
//    }

//    void setUAxis(QVector3D const& uaxis)
//    {
//      m_plane->m_u = utils::toGMVectorH4D(uaxis);
//      emit uAxisChanged(uaxis);
//      updateAndMarkDirty();
//    }

//    void setVAxis(QVector3D const& vaxis)
//    {
//      m_plane->m_v = utils::toGMVectorH4D(vaxis);
//      emit vAxisChanged(vaxis);
//      updateAndMarkDirty();
//    }

    using TensorProductSurfaceGeometry::SurfaceType;
    SurfaceType* internalSurface() const override { return m_bezsurf.get(); }

//  signals:
//    void originChanged( QVector3D origin );
//    void uAxisChanged( QVector3D uAxis );
//    void vAxisChanged( QVector3D vAxis );


//  private:
    std::unique_ptr<BezierSurfaceType> m_bezsurf;
  };

}   // namespace gmqt


#endif   // GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_BEZIERSURFACEGEOMETRY_H
