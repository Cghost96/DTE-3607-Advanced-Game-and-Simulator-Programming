#ifndef GMLIBQTINTEGRATION_GEOMETRY_POLYGONALSURFACE_POLYGONSURFACEGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_POLYGONALSURFACE_POLYGONSURFACEGEOMETRY_H


#include "../../polygonalsurfacegeometry.h"

// gmlib
#include "gmlib/parametric/polygonalsurface_constructions/polygonsurface.h"


namespace gmqt
{

  class PolygonSurfaceGeometry : public PolygonalSurfaceGeometry {
    Q_OBJECT


    using PolygonSurfaceType = gm::parametric::PolygonSurface<
      gm::spaces::ProjectiveSpace<double, 3>>;


    std::unique_ptr<PolygonSurfaceType> m_surface;

  public:
    PolygonSurfaceGeometry(QQuick3DObject* parent = nullptr)
      : PolygonalSurfaceGeometry(parent)
    {

      using Point    = PolygonSurfaceType::Point;
      using Vertices = PolygonSurfaceType::Vertices;

      auto const V = Vertices{Point{-5.0, 0.0, -5.0}, Point{0.0, 0.0, -5.0},
                              Point{5.0, 0.0, 0.0}, Point{5.0, 0.0, 5.0},
                              Point{-5.0, 0.0, 0.0}};

      m_surface = std::make_unique<PolygonSurfaceType>(V);
      updateAndMarkDirty();
    }


    using PolygonalSurfaceGeometry::SurfaceType;
    SurfaceType* internalSurface() const override { return m_surface.get(); }
  };



}   // namespace gmqt

#endif // GMLIBQTINTEGRATION_GEOMETRY_POLYGONALSURFACE_POLYGONSURFACEGEOMETRY_H
