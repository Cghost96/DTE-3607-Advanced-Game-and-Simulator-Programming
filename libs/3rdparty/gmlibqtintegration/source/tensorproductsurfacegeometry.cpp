#include <gmlibqtintegration/geometry/tensorproductsurfacegeometry.h>

#include <gmlibqtintegration/private/cpp_helpers.h>
#include <gmlibqtintegration/private/surfacegeometrybuilder.h>

namespace gmqt
{



  TensorProductSurfaceGeometry::TensorProductSurfaceGeometry(
    QQuick3DObject* parent)
    : CustomQuick3DGeometry(parent)
  {
  }

  QSize TensorProductSurfaceGeometry::noSamples() const { return m_no_samples; }

  void TensorProductSurfaceGeometry::setNoSamples(QSize const& no_samples)
  {
    m_no_samples = no_samples;
    emit noSamplesChanged(no_samples);
    updateAndMarkDirty();
  }

  void TensorProductSurfaceGeometry::rebuildGeometry()
  {
    auto sampling_result = sample(m_no_samples);
    auto [vertex_data, bmin, bmax]
      = detail::surfacegeometry::buildTPSurfaceGeometryStandaloneVertexBuffer(
        sampling_result);

    clear();

    setVertexData(vertex_data);

    detail::setPTeNTaBQQuick3DGeomertryAttributes(*this);
    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Triangles);
    setStride(detail::surfacegeometry::tpSurfaceVertexElementSize());

    setBounds(bmin, bmax);
  }

  TensorProductSurfaceGeometry::SurfaceSamplingResult
  TensorProductSurfaceGeometry::sample(const QSize& no_samples) const
  {
    auto& surface = *internalSurface();

    auto sampling_result = gm::parametric::tensorproductsurface::sample(
      surface, {surface.domain().U.start(), surface.domain().V.start()},
      {surface.domain().U.end(), surface.domain().V.end()},
      {size_t(no_samples.width()), size_t(no_samples.height())}, {1ul, 1ul});

    return sampling_result;
  }


}   // namespace gmqt
