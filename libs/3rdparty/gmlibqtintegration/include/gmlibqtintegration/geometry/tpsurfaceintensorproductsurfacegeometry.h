#ifndef GMLIBQTINTEGRATION_GEOMETRY_TPSURFACEINTENSORPRODUCTSURFACEGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_TPSURFACEINTENSORPRODUCTSURFACEGEOMETRY_H


#include "tensorproductsurfacegeometry.h"


namespace gmqt
{

  class TPSurfaceInTensorProductSurfaceGeometry : public TensorProductSurfaceGeometry {
    Q_OBJECT

    Q_PROPERTY(
      TensorProductSurfaceGeometry* embedGeometry READ embedGeometry WRITE
        setEmbedGeometry NOTIFY embedGeometryChanged)

  public:
    using TensorProductSurfaceGeometry::SurfaceType;

    TPSurfaceInTensorProductSurfaceGeometry(QQuick3DObject* parent = nullptr)
      : TensorProductSurfaceGeometry(parent)
    {
    }

    QPointer<TensorProductSurfaceGeometry> m_embed_surface{nullptr};


  private:
    TensorProductSurfaceGeometry* embedGeometry() const
    {
      return m_embed_surface;
    }

    void setEmbedGeometry(TensorProductSurfaceGeometry* embed_surface)
    {
      m_embed_surface = embed_surface;
      updateInternalEmbedSurface();
      connect(
        m_embed_surface, &TensorProductSurfaceGeometry::geometryNodeDirty, this,
        &TPSurfaceInTensorProductSurfaceGeometry::updateInternalEmbedSurface);
    }

    virtual void updateInternalEmbedSurface() = 0;

  signals:
    void embedGeometryChanged( TensorProductSurfaceGeometry* embedSurface );
  };


}   // namespace gmqt


#endif   // GMLIBQTINTEGRATION_GEOMETRY_TPSURFACEINTENSORPRODUCTSURFACEGEOMETRY_H
