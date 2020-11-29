#ifndef GMLIBQTINTEGRATION_GEOMETRY_POLYGONALSURFACEGEOMETRIES_H
#define GMLIBQTINTEGRATION_GEOMETRY_POLYGONALSURFACEGEOMETRIES_H

#include "../../polygonalsurfacegeometry.h"
#include "polygonsurfacegeometry.h"
#include "gbpatchgeometry.h"

namespace gmqt
{

  inline void registerCustomGMlib2PolygonalSurfaceGeometryQmlTypes()
  {
    CustomQuick3DGeometry::registerQmlType<PolygonSurfaceGeometry>("PolygonSurfaceGeometry");
    CustomQuick3DGeometry::registerQmlType<GBPatchGeometry>("GBPatchGeometry");
  }


}   // namespace gmqt

#endif // GMLIBQTINTEGRATION_GEOMETRY_POLYGONALSURFACEGEOMETRIES_H
