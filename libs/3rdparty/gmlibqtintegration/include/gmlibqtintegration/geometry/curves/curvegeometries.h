#ifndef GMLIBQTINTEGRATION_GEOMETRY_CURVEGEOMETRIES_H
#define GMLIBQTINTEGRATION_GEOMETRY_CURVEGEOMETRIES_H

#include "../tensorproductsurfacegeometry.h"
#include "../polygonalsurfacegeometry.h"

#include "linegeometry.h"
#include "lineintensorproductsurfacegeometry.h"
#include "lineinpolygonalsurfacegeometry.h"


namespace gmqt
{


  inline void registerCustomGMlib2CurveGeometryQmlTypes()
  {
    // Curve geometry
    CustomQuick3DGeometry::registerUncreatableQmlType<CurveGeometry>(
      "CurveGeometry");
    CustomQuick3DGeometry::registerQmlType<LineGeometry>("LineGeometry");

    // Curve-in-tpsurface geometry
    CustomQuick3DGeometry::registerUncreatableQmlType<
      CurveInTensorProductSurfaceGeometry>(
      "CurveInTensorProductSurfaceGeometry");
    CustomQuick3DGeometry::registerQmlType<LineInTensorProductSurfaceGeometry>(
      "LineInTensorProductSurfaceGeometry");

    // Curve-in-polygonalsurface geometry
    CustomQuick3DGeometry::registerUncreatableQmlType<
      CurveInPolygonalSurfaceGeometry>("CurveInPolygonalSurfaceGeometry");
    CustomQuick3DGeometry::registerQmlType<LineInPolygonalSurfaceGeometry>(
      "LineInPolygonalSurfaceGeometry");
  }


}   // namespace gmqt

#endif // GMLIBQTINTEGRATION_GEOMETRY_CURVEGEOMETRIES_H
