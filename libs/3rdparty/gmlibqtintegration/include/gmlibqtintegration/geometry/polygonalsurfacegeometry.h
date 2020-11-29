#ifndef GMLIBQTINTEGRATION_GEOMETRY_POLYGONALSURFACE_H
#define GMLIBQTINTEGRATION_GEOMETRY_POLYGONALSURFACE_H

// integration
#include "../private/cpp_helpers.h"
#include "../private/customquick3dgeometry.h"
#include "../private/surfacegeometrybuilder.h"


// gmlib2
#include <gmlib/parametric/polygonalsurface.h>

// gmconcepts
#include <gmconcepts/manifolds.h>

// qt
#include <QQuick3DGeometry>
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>
#include <QSize>

// stl
#include <string>

namespace gmqt
{


  class PolygonalSurfaceGeometry : public CustomQuick3DGeometry {
    Q_OBJECT

    Q_PROPERTY(int samplingRings READ noSamplingRings WRITE setNoSamplingRings
                 NOTIFY noSamplingRingsChanged)

  public:
    PolygonalSurfaceGeometry(QQuick3DObject* parent = nullptr);


    int noSamplingRings() const;

    void setNoSamplingRings(int no_sampling_rings);


    using EmbedSpaceType = gm::spaces::ProjectiveSpace<double, 3ul>;
    using SurfaceType    = gm::parametric::PolygonalSurface<
      gm::parametric::mappingkernel::MVCPolygonalSurfaceMappingKernel,
      EmbedSpaceType>;
    virtual SurfaceType* internalSurface() const { return nullptr; }

  signals:
    void noSamplingRingsChanged(int no_sampling_rings);

  private:
    int m_no_sampling_rings{4};


    void rebuildGeometry() override final;

  protected:
    using SamplingResult
      = detail::surfacegeometry::PolygonalSurfaceSamplingResult;
    using FaceIndices
      = detail::surfacegeometry::PolygonalSurfaceSamplingResultFaceIndices;

  private:
    virtual std::pair<SamplingResult, FaceIndices>
    sample(int no_sampling_rings) const;
  };





}   // namespace gmqt


#endif   // GMLIBQTINTEGRATION_GEOMETRY_POLYGONALSURFACE_H
