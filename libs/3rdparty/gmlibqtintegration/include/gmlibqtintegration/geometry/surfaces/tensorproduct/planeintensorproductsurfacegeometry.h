#ifndef GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_PLANEINTENSORPRODUCTSURFACEGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_PLANEINTENSORPRODUCTSURFACEGEOMETRY_H


#include "../../tpsurfaceintensorproductsurfacegeometry.h"

// gmlib
#include "gmlib/parametric/subobject_constructions/subtensorproductsurface.h"


namespace gmqt
{

  class PlaneInTensorProductSurfaceGeometry
    : public TPSurfaceInTensorProductSurfaceGeometry {
    Q_OBJECT

    Q_PROPERTY(
      QVector2D origin READ origin WRITE setOrigin NOTIFY originChanged)
    Q_PROPERTY(QVector2D uAxis READ uAxis WRITE setUAxis NOTIFY uAxisChanged)
    Q_PROPERTY(QVector2D vAxis READ vAxis WRITE setVAxis NOTIFY vAxisChanged)

  public:
//    using SurfaceInSurfaceType::SurfaceType;
    using TPSurfaceInTensorProductSurfaceType
      = gm::parametric::TPSurfaceInTensorProductSurface<
        gm::parametric::Plane, SurfaceType::EmbedSpace>;
    PlaneInTensorProductSurfaceGeometry(QQuick3DObject* parent = nullptr)
      : TPSurfaceInTensorProductSurfaceGeometry(parent)
    {
    }

    QVector2D origin() const { return m_origin; }
    QVector2D uAxis() const { return m_uaxis; }
    QVector2D vAxis() const { return m_vaxis; }

    void setOrigin(QVector2D const& origin)
    {
      m_origin = origin;
      emit originChanged(origin);
      updateInternalEmbedSurface();
    }

    void setUAxis(QVector2D const& uaxis)
    {
      m_uaxis = uaxis;
      emit uAxisChanged(uaxis);
      updateInternalEmbedSurface();
    }

    void setVAxis(QVector2D const& vaxis)
    {
      m_vaxis = vaxis;
      emit vAxisChanged(vaxis);
      updateInternalEmbedSurface();
    }

  signals:
    void originChanged( QVector2D origin );
    void uAxisChanged( QVector2D uAxis );
    void vAxisChanged( QVector2D vAxis );


  private:
    QVector2D m_origin{0, 0};
    QVector2D m_uaxis{10, 0};
    QVector2D m_vaxis{0, 10};

    std::unique_ptr<TPSurfaceInTensorProductSurfaceType> m_surfaceinsurface{nullptr};

    void updateInternalEmbedSurface() override {

      if (not m_embed_surface) return;

      auto* internal_surface = m_embed_surface->internalSurface();
      if (not internal_surface) return;

      if( not m_surfaceinsurface ) {

        auto sub_p = utils::toGMVectorT(m_origin);
        auto sub_uaxis = utils::toGMVectorT(m_uaxis);
        auto sub_vaxis = utils::toGMVectorT(m_vaxis);
        m_surfaceinsurface
          = std::make_unique<TPSurfaceInTensorProductSurfaceType>(
            *internal_surface, sub_p, sub_uaxis, sub_vaxis);
      }
      else {

        auto sub_p = utils::toGMPointH3D(m_origin);
        auto sub_uaxis = utils::toGMVectorH3D(m_uaxis);
        auto sub_vaxis = utils::toGMVectorH3D(m_vaxis);
        m_surfaceinsurface->m_msurface.m_pt = sub_p;
        m_surfaceinsurface->m_msurface.m_u = sub_uaxis;
        m_surfaceinsurface->m_msurface.m_v = sub_vaxis;
      }
      updateAndMarkDirty();
    }


    SurfaceType* internalSurface() const override
    {
      return m_surfaceinsurface.get();
    }
  };

}   // namespace gmqt


#endif   // GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACE_PLANEINTENSORPRODUCTSURFACEGEOMETRY_H
