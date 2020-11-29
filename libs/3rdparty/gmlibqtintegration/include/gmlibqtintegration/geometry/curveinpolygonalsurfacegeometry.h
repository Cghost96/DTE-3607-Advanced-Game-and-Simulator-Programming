#ifndef GMLIBQTINTEGRATION_GEOMETRY_CURVEINPOLYGONALSURFACEGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_CURVEINPOLYGONALSURFACEGEOMETRY_H


#include "curvegeometry.h"
#include "polygonalsurfacegeometry.h"


namespace gmqt
{

  class CurveInPolygonalSurfaceGeometry : public CurveGeometry {
    Q_OBJECT

    Q_PROPERTY(
      PolygonalSurfaceGeometry* embedGeometry READ embedGeometry WRITE
        setEmbedGeometry NOTIFY embedGeometryChanged)

  public:

    using SurfaceType = PolygonalSurfaceGeometry::SurfaceType;

    CurveInPolygonalSurfaceGeometry(QQuick3DObject* parent = nullptr)
      : CurveGeometry(parent)
    {
    }

    QPointer<PolygonalSurfaceGeometry> m_surfacegeometry{nullptr};


  private:
    PolygonalSurfaceGeometry* embedGeometry() const
    {
      return m_surfacegeometry;
    }

    void setEmbedGeometry(PolygonalSurfaceGeometry* surface_geometry)
    {
      m_surfacegeometry = surface_geometry;
      updateInternalSubCurve();
      connect(m_surfacegeometry, &PolygonalSurfaceGeometry::geometryNodeDirty,
              this, &CurveInPolygonalSurfaceGeometry::updateInternalSubCurve);
    }

    virtual void updateInternalSubCurve() = 0;

  signals:
    void embedGeometryChanged(PolygonalSurfaceGeometry* tpSurfaceGeometry);
  };



}   // namespace gmqt


#endif   // GMLIBQTINTEGRATION_GEOMETRY_CURVEINPOLYGONALSURFACEGEOMETRY_H
