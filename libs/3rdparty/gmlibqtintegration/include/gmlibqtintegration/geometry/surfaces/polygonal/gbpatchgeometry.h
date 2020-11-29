#ifndef GMLIBQTINTEGRATION_GEOMETRY_POLYGONALSURFACE_GBPATCHGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_POLYGONALSURFACE_GBPATCHGEOMETRY_H


#include "../../polygonalsurfacegeometry.h"

// gmlib
#include "gmlib/parametric/polygonalsurface_constructions/generalizedbezierpatch.h"


namespace gmqt
{

  class GBPatchGeometry : public PolygonalSurfaceGeometry {
    Q_OBJECT


    using GBPatchType = gm::parametric::GeneralizedBezierPatch<
      gm::spaces::ProjectiveSpace<double, 3>>;


    std::unique_ptr<GBPatchType> m_gbpatch;

  public:
    GBPatchGeometry(QQuick3DObject* parent = nullptr)
      : PolygonalSurfaceGeometry(parent)
    {

      using Point   = GBPatchType::Point;
//      using DVector = GBPatchType::DVector;

      const auto P = gm::DVectorT<Point>{
        Point{-5.0, 0.0, -5.0}, Point{0.0, 0.0, -5.0}, Point{5.0, 0.0, 0.0},
        Point{5.0, 0.0, 5.0}, Point{-5.0, 0.0, 0.0}};

      auto controlnets
        = gm::parametric::generalizedbezierpatch::constructControlNets<
          GBPatchType>(P, 5);
      controlnets[0](2, 2)[1] = 10.0;
      controlnets[2](2, 1)[1] = 0.5;
      controlnets[4](1, 1)[1] = 8.0;

      m_gbpatch = std::make_unique<GBPatchType>(controlnets);
      updateAndMarkDirty();
    }


    using PolygonalSurfaceGeometry::SurfaceType;
    SurfaceType* internalSurface() const override { return m_gbpatch.get(); }

//  private:
//    using PolygonalSurfaceGeometry::SamplingResult;
//    std::pair<SamplingResult, FaceIndices>
//    sample(int no_sampling_rings) const override
//    {

//      auto* polygon = internalSurface();
//      if (not polygon) return {};


//      auto sampling_positions = gm::parametric::polygonsurface::
//        convex_algorithms::generateTriSamplingPSpacePositions(
//          size_t(no_sampling_rings), polygon->domain().polygon);
//      auto sampling_result = gm::parametric::polygonsurface::sample(
//        *polygon, sampling_positions, {{{{1, 0}}, {{0, 1}}}});

//      auto trifaceindices = gm::parametric::polygonsurface::
//        convex_algorithms::generateTriSamplingFaceIndices(
//          size_t(no_sampling_rings), polygon->domain().polygon);

//      return std::pair(sampling_result, trifaceindices);
//    }
  };



}   // namespace gmqt

#endif // GMLIBQTINTEGRATION_GEOMETRY_POLYGONALSURFACE_GBPATCHGEOMETRY_H
