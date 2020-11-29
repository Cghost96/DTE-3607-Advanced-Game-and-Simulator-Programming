#ifndef GMLIBQTINTEGRATION_GEOMETRY_CURVEINTENSORPRODUCTSURFACEGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_CURVEINTENSORPRODUCTSURFACEGEOMETRY_H


#include "curvegeometry.h"
#include "tensorproductsurfacegeometry.h"


namespace gmqt
{

  class CurveInTensorProductSurfaceGeometry : public CurveGeometry {
    Q_OBJECT

    Q_PROPERTY(TensorProductSurfaceGeometry* embedGeometry READ embedGeometry
                 WRITE setEmbedGeometry NOTIFY embedGeometryChanged)

  public:
    using SurfaceType = TensorProductSurfaceGeometry::SurfaceType;

    CurveInTensorProductSurfaceGeometry(QQuick3DObject* parent = nullptr)
      : CurveGeometry(parent)
    {
    }

    QPointer<TensorProductSurfaceGeometry> m_surfacegeometry{nullptr};


  private:
    TensorProductSurfaceGeometry* embedGeometry() const
    {
      return m_surfacegeometry;
    }

    void setEmbedGeometry(TensorProductSurfaceGeometry* surface_geometry)
    {
      m_surfacegeometry = surface_geometry;
      updateInternalSubCurve();
      connect(m_surfacegeometry,
              &TensorProductSurfaceGeometry::geometryNodeDirty, this,
              &CurveInTensorProductSurfaceGeometry::updateInternalSubCurve);
    }

    virtual void updateInternalSubCurve() = 0;

  signals:
    void embedGeometryChanged(TensorProductSurfaceGeometry* tpSurfaceGeometry);
  };


}   // namespace gmqt


#endif   // GMLIBQTINTEGRATION_GEOMETRY_CURVEINTENSORPRODUCTSURFACEGEOMETRY_H
