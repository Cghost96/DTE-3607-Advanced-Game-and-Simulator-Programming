#include <gmlibqtintegration/geometry/curvegeometry.h>

#include <gmlibqtintegration/private/cpp_helpers.h>
#include <gmlibqtintegration/private/surfacegeometrybuilder.h>

namespace gmqt
{



  CurveGeometry::CurveGeometry(QQuick3DObject* parent)
    : CustomQuick3DGeometry(parent)
  {
  }

  int CurveGeometry::noSamples() const { return m_no_samples; }

  int CurveGeometry::noSlices() const { return m_no_slices; }

  float CurveGeometry::curveTubeRadius() const { return m_curve_tube_radius; }

  void CurveGeometry::setNoSamples(int no_samples)
  {
    m_no_samples = no_samples;
    emit noSamplesChanged(no_samples);
    updateAndMarkDirty();
  }

  void CurveGeometry::setNoSlices(int no_slices)
  {
    m_no_slices = no_slices;
    emit noSlicesChanged(no_slices);
    updateAndMarkDirty();
  }

  void CurveGeometry::setCurveTubeRadius(float radius)
  {
    m_curve_tube_radius = radius;
    emit curveTubeRadiusChanged(radius);
    updateAndMarkDirty();
  }


  void CurveGeometry::rebuildGeometry()
  {
    auto sampling_result
      = sampleAsTube(int(m_no_samples), int(m_no_slices), float(m_curve_tube_radius));
    if (not sampling_result) return;


    auto const& [vertex_data, bmin, bmax]
      = detail::surfacegeometry::buildTPSurfaceGeometryStandaloneVertexBuffer(
        sampling_result.value());

    clear();

    setVertexData(vertex_data);

    detail::setPTeNTaBQQuick3DGeomertryAttributes(*this);
    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Triangles);
    setStride(detail::surfacegeometry::tpSurfaceVertexElementSize());

    setBounds(bmin, bmax);
  }

  CurveGeometry::SurfaceSamplingResult
  CurveGeometry::sampleAsTube(int no_samples, int no_slices, float radius) const
  {

    auto* curve = internalCurve();
    if (!curve) return {};


    auto sampling_result = gm::parametric::curve::sampleAsTube(
      *curve, curve->domain().I, size_t(no_samples), size_t(no_slices),
      double(radius));

    return {sampling_result};
  }


  }   // namespace gmqt
