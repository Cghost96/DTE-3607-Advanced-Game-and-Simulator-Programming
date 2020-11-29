#include <gmlibqtintegration/geometry/polygonalsurfacegeometry.h>

#include <gmlibqtintegration/private/cpp_helpers.h>
#include <gmlibqtintegration/private/surfacegeometrybuilder.h>

namespace gmqt
{

  PolygonalSurfaceGeometry::PolygonalSurfaceGeometry(QQuick3DObject* parent)
    : CustomQuick3DGeometry(parent)
  {
  }

  int PolygonalSurfaceGeometry::noSamplingRings() const
  {
    return m_no_sampling_rings;
  }

  void PolygonalSurfaceGeometry::setNoSamplingRings(int no_sampling_rings)
  {
    m_no_sampling_rings = no_sampling_rings;
    emit noSamplingRingsChanged(no_sampling_rings);
    updateAndMarkDirty();
  }


  void PolygonalSurfaceGeometry::rebuildGeometry()
  {
    auto [sampling_result, indices] = sample(m_no_sampling_rings);
    auto [vertex_data, bmin, bmax]  = detail::surfacegeometry::
      buildPolygonalSurfaceGeometryStandaloneVertexBuffer(sampling_result,
                                                          indices);

    clear();

    setVertexData(vertex_data);

    detail::setPNBQQuick3DGeomertryAttributes(*this);
    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Triangles);
    setStride(detail::surfacegeometry::polygonSurfaceVertexElementSize());

    setBounds(bmin, bmax);
  }

  std::pair<PolygonalSurfaceGeometry::SamplingResult,
            PolygonalSurfaceGeometry::FaceIndices>
  PolygonalSurfaceGeometry::sample(int no_sampling_rings) const
  {
    auto* polygon = internalSurface();
    if (not polygon) return {};


    auto sampling_positions = gm::parametric::polygonsurface::
      convex_algorithms::generateTriSamplingPSpacePositions(
        size_t(no_sampling_rings), polygon->domain().polygon);

    using DVector = typename SurfaceType::DVector;
    auto sampling_result = gm::parametric::polygonsurface::sample(
          *polygon, sampling_positions, {{DVector{1,0},DVector{0,1}}}
          );

    auto trifaceindices = gm::parametric::polygonsurface::
      convex_algorithms::generateTriSamplingFaceIndices(
        size_t(no_sampling_rings), polygon->domain().polygon);

    return std::pair(sampling_result, trifaceindices);
  }

}   // namespace gmqt
