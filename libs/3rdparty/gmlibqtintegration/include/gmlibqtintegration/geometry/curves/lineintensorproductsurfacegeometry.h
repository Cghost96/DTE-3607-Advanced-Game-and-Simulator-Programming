#ifndef GMLIBQTINTEGRATION_GEOMETRY_CURVES_LINEINTENSORPRODUCTSURFACEGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_CURVES_LINEINTENSORPRODUCTSURFACEGEOMETRY_H


#include "../curveintensorproductsurfacegeometry.h"

// gmlib
#include <gmlib/parametric/subobject_constructions/subcurve.h>


namespace gmqt
{

  class LineInTensorProductSurfaceGeometry
    : public CurveInTensorProductSurfaceGeometry {
    Q_OBJECT

    Q_PROPERTY(
      QVector2D origin READ origin WRITE setOrigin NOTIFY originChanged)
    Q_PROPERTY(QVector2D direction READ direction WRITE setDirection NOTIFY
                 directionChanged)

  public:
    using CurveInTensorProductSurfaceGeometry::SurfaceType;
    using CurveInTensorProductSurfaceType = gm::parametric::CurveInTensorProductSurface<
      gm::parametric::Line, SurfaceType::EmbedSpace>;

    LineInTensorProductSurfaceGeometry(QQuick3DObject* parent = nullptr)
      : CurveInTensorProductSurfaceGeometry(parent)
    {
    }

    QVector2D origin() const { return m_origin; }
    QVector2D direction() const { return m_direction; }

    void setOrigin(QVector2D origin) {
      m_origin = origin;
      emit originChanged(origin);
      updateInternalSubCurve();
    }
    void setDirection(QVector2D direction) {
      m_direction = direction;
      emit directionChanged(direction);
      updateInternalSubCurve();
    }

  signals:
    void originChanged(QVector2D origin);
    void directionChanged(QVector2D direction);

  private:
    QVector2D m_origin {0,0};
    QVector2D m_direction {1,0};

    std::unique_ptr<CurveInTensorProductSurfaceType> m_subcurve{nullptr};

    void updateInternalSubCurve() override {

      if (not m_surfacegeometry) return;

      auto* internal_surface = m_surfacegeometry->internalSurface();
      if (not internal_surface) return;

      if( not m_subcurve ) {

        auto subcurve_p = utils::toGMVectorT(m_origin);
        auto subcurve_v = utils::toGMVectorT(m_direction);
        m_subcurve      = std::make_unique<CurveInTensorProductSurfaceType>(
          *internal_surface, subcurve_p, subcurve_v);
      }
      else {

        auto subcurve_p = utils::toGMPointH3D(m_origin);
        auto subcurve_v = utils::toGMVectorH3D(m_direction);
        m_subcurve->m_mcurve.m_pt = subcurve_p;
        m_subcurve->m_mcurve.m_v = subcurve_v;
      }
      updateAndMarkDirty();
    }


    CurveType* internalCurve() const override { return m_subcurve.get(); }
  };



}   // namespace gmqt




#endif // GMLIBQTINTEGRATION_GEOMETRY_CURVES_LINEINTENSORPRODUCTSURFACEGEOMETRY_H
