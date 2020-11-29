#ifndef GMLIBQTINTEGRATION_GEOMETRY_CURVEGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_CURVEGEOMETRY_H

// integration
#include "../private/customquick3dgeometry.h"
#include "../private/cpp_helpers.h"

// gmlib2
#include <gmlib/parametric/classic_objects/line.h>

// gmconcepts
#include <gmconcepts/manifolds.h>


// stl
#include <string>
#include <memory>
#include <optional>

namespace gmqt
{


  class CurveGeometry : public CustomQuick3DGeometry {
    Q_OBJECT

    Q_PROPERTY(
      int samples READ noSamples WRITE setNoSamples NOTIFY noSamplesChanged)
    Q_PROPERTY(
      int slices READ noSlices WRITE setNoSlices NOTIFY noSlicesChanged)
    Q_PROPERTY(float curveTubeRadius READ curveTubeRadius WRITE
                 setCurveTubeRadius NOTIFY curveTubeRadiusChanged)

  public:
    CurveGeometry(QQuick3DObject* parent = nullptr);

    int   noSamples() const;
    int   noSlices() const;
    float curveTubeRadius() const;

    void setNoSamples( int no_samples );
    void setNoSlices( int no_slices );
    void setCurveTubeRadius( float radius );

    using EmbedSpaceType = gm::spaces::ProjectiveSpace<double,3ul>;
    using CurveType      = gm::parametric::Curve<EmbedSpaceType>;


    virtual CurveType* internalCurve() const { return m_curve; }
    void               setInternalCurve(CurveType* ptr) { m_curve = ptr; }


  signals:
    void noSamplesChanged( int no_samples );
    void noSlicesChanged( int no_slices );
    void curveTubeRadiusChanged( float radius );

  private:
    int        m_no_samples{20};
    int        m_no_slices{5};
    float      m_curve_tube_radius{2.0f};
    CurveType* m_curve;

    void rebuildGeometry() override final;

  protected:
    using SurfaceSamplingResult = std::optional<
      gm::datastructures::parametrics::tensorproductsurface::SamplingResult<
        gm::VectorT<double, 4>>>;

  private:
    virtual SurfaceSamplingResult
    sampleAsTube(int no_samples, int no_slices, float radius) const;
  };







}   // namespace gmqt


#endif   // GMLIBQTINTEGRATION_GEOMETRY_CURVEGEOMETRY_H
