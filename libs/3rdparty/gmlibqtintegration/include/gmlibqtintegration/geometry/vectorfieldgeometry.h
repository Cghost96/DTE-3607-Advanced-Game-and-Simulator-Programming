#ifndef GMLIBQTINTEGRATION_GEOMETRY_VECTORFIELDGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_VECTORFIELDGEOMETRY_H

// integration
#include "../private/customquick3dgeometry.h"
#include "../private/cpp_helpers.h"

#include "curvegeometry.h"
#include "tensorproductsurfacegeometry.h"
#include "polygonalsurfacegeometry.h"


namespace gmqt
{

  class VectorFieldGeometry : public CustomQuick3DGeometry {
    Q_OBJECT

    Q_PROPERTY(float radius READ radius WRITE setRadius NOTIFY radiusChanged)
    Q_PROPERTY(CurveGeometry* curveGeometry READ curveGeometry WRITE
                 setCurveGeometry NOTIFY curveGeometryChanged)
    Q_PROPERTY(
      TensorProductSurfaceGeometry* tpSurfaceGeometry READ tpSurfaceGeometry
        WRITE setTpSurfaceGeometry NOTIFY tpSurfaceGeometryChanged)

    Q_PROPERTY(PolygonalSurfaceGeometry* polygonalSurfaceGeometry READ
                 polygonalSurfaceGeometry WRITE setPolygonalSurfaceGeometry
                   NOTIFY polygonalSurfaceGeometryChanged)

  public:
    VectorFieldGeometry(QQuick3DObject* parent = nullptr);

    float radius() const;
    void  setRadius(float radius);

    CurveGeometry* curveGeometry() const;
    void setCurveGeometry( CurveGeometry* curve_geometry );

    TensorProductSurfaceGeometry* tpSurfaceGeometry() const;
    void setTpSurfaceGeometry(TensorProductSurfaceGeometry* tpsurf_geometry);

    PolygonalSurfaceGeometry* polygonalSurfaceGeometry() const;
    void
    setPolygonalSurfaceGeometry(PolygonalSurfaceGeometry* polysurf_geometry);

  signals:
    void radiusChanged( float radius );
    void curveGeometryChanged( CurveGeometry* curveGeometry );
    void tpSurfaceGeometryChanged(TensorProductSurfaceGeometry* tpSurfGeometry);
    void
    polygonalSurfaceGeometryChanged(PolygonalSurfaceGeometry* polySurfGeometry);

  private:
    float                                  m_radius{1.0f};

    QPointer<CurveGeometry>                m_curve_geometry{nullptr};
    QPointer<TensorProductSurfaceGeometry> m_tpsurface_geometry{nullptr};
    QPointer<PolygonalSurfaceGeometry>     m_psurface_geometry{nullptr};


    void resetInputVfType();



    struct VertexElement {
      QVector4D position;
    };
    struct LineElement {
      VertexElement p0;
      VertexElement p1;
    };

    void rebuildGeometry() override final;
    void updateGeometryData(
      std::tuple<QByteArray, QVector3D, QVector3D> const& geometry_data);

    std::tuple<QByteArray, QVector3D, QVector3D> buildCurveVertexData() const;
    std::tuple<QByteArray, QVector3D, QVector3D>
    buildTpSurfaceVertexData() const;
    std::tuple<QByteArray, QVector3D, QVector3D>
    buildPolygonalSurfaceVertexData() const;
  };



  inline void registerCustomGMlib2UtilsGeometryQmlTypes()
  {
    CustomQuick3DGeometry::registerQmlType<VectorFieldGeometry>(
      "VectorFieldGeometryTmp");
  }


}   // namespace gmqt


#endif   // GMLIBQTINTEGRATION_GEOMETRY_VECTORFIELDGEOMETRY_H
