#ifndef GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACEGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACEGEOMETRY_H

// integration
#include "../private/customquick3dgeometry.h"
#include "../private/cpp_helpers.h"

// gmlib2
#include <gmlib/parametric/classic_objects/torus.h>
#include <gmlib/parametric/classic_objects/plane.h>

// gmconcepts
#include <gmconcepts/manifolds.h>

// qt
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>
#include <QSize>

// stl
#include <string>

namespace gmqt
{


  class TensorProductSurfaceGeometry : public CustomQuick3DGeometry {
    Q_OBJECT

    Q_PROPERTY(
      QSize samples READ noSamples WRITE setNoSamples NOTIFY noSamplesChanged)

  public:
    TensorProductSurfaceGeometry(QQuick3DObject* parent = nullptr);

    QSize noSamples() const;

    void setNoSamples(QSize const& no_samples);

    using EmbedSpaceType = gm::spaces::ProjectiveSpace<double, 3ul>;
    using SurfaceType    = gm::parametric::TensorProductSurface<EmbedSpaceType>;
    virtual SurfaceType* internalSurface() const { return nullptr; }


  signals:
    void noSamplesChanged(QSize no_samples);

  protected:
    using SurfaceSamplingResult = gm::datastructures::parametrics::
      tensorproductsurface::SamplingResult<gm::VectorT<double, 4>>;

    QSize m_no_samples{10, 10};

    void                          rebuildGeometry() override final;
    virtual SurfaceSamplingResult sample(QSize const& no_samples) const;
  };





}   // namespace gmqt


#endif   // GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACEGEOMETRY_H
