#ifndef GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACEGEOMETRIES_H
#define GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACEGEOMETRIES_H

#include "../../tensorproductsurfacegeometry.h"
#include "../../tpsurfaceintensorproductsurfacegeometry.h"

#include "planegeometry.h"
#include "torusgeometry.h"
#include "spheregeometry.h"
#include "planeintensorproductsurfacegeometry.h"


// gmlib2
#include <gmlib/parametric/subobject_constructions/subtensorproductsurface.h>

namespace gmqt
{

  inline void registerCustomGMlib2TPSurfaceGeometryQmlTypes()
  {
    CustomQuick3DGeometry::registerUncreatableQmlType<
      TensorProductSurfaceGeometry>("TensorProductSurfaceGeometry");
    CustomQuick3DGeometry::registerQmlType<PlaneGeometry>("PlaneGeometry");
    CustomQuick3DGeometry::registerQmlType<SphereGeometry>("SphereGeometry");
    CustomQuick3DGeometry::registerQmlType<TorusGeometry>("TorusGeometry");

    CustomQuick3DGeometry::registerUncreatableQmlType<
      TPSurfaceInTensorProductSurfaceGeometry>("TPSurfaceInTensorProductSurfaceGeometry");
    CustomQuick3DGeometry::registerQmlType<PlaneInTensorProductSurfaceGeometry>(
      "PlaneInTensorProductSurfaceGeometry");
  }


}   // namespace gmqt

#endif // GMLIBQTINTEGRATION_GEOMETRY_TENSORPRODUCTSURFACEGEOMETRIES_H
