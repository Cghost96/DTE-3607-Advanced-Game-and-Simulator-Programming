#include "limitedplanegeometry.h"


// gmlib qt intergration
#include <gmlibqtintegration/private/surfacegeometrybuilder.h>
#include <gmlibqtintegration/private/cpp_helpers.h>

// gmlib
#include <gmlib/parametric/classic_objects/plane.h>


// qt
#include <QByteArray>
#include <QVector3D>


// stl
#include <limits>
#include <memory>

namespace app
{

  LimitedPlaneGeometry::LimitedPlaneGeometry(QQuick3DObject* parent)
    : CustomQuick3DGeometry(parent)
  {
  }

  void LimitedPlaneGeometry::set(Point const& p, Vector const& u,
                                 Vector const& v, Type const& thickness)
  {
    m_p         = p;
    m_u         = u;
    m_v         = v;
    m_thickness = thickness;
    sample();
  }

  void LimitedPlaneGeometry::sample() { updateAndMarkDirty(); }

  void LimitedPlaneGeometry::rebuildGeometry()
  {


    using NLF = std::numeric_limits<float>;
    QByteArray vertex_data;
    QVector3D  bmin{NLF::max(), NLF::max(), NLF::max()};
    QVector3D  bmax{NLF::min(), NLF::min(), NLF::min()};

    auto appendPlaneVertexData = [&vertex_data, &bmin, &bmax](auto const& p,
                                                              auto const& u,
                                                              auto const& v) {
      auto plane   = std::make_unique<PlaneType>(p, u, v);
      auto samples = gm::parametric::tensorproductsurface::sample(
        *plane, {plane->domain().U.start(), plane->domain().V.start()},
        {plane->domain().U.end(), plane->domain().V.end()},
        {size_t(10), size_t(10)}, {1ul, 1ul});


      auto [vertex_data_part, bmin_part, bmax_part] = gmqt::detail::
        surfacegeometry::buildTPSurfaceGeometryStandaloneVertexBuffer(samples);

      vertex_data.append(vertex_data_part);
      bmin.setX(std::min(bmin.x(), bmin_part.x()));
      bmin.setY(std::min(bmin.y(), bmin_part.y()));
      bmin.setY(std::min(bmin.z(), bmin_part.z()));
      bmax.setX(std::max(bmax.x(), bmax_part.x()));
      bmax.setY(std::max(bmax.y(), bmax_part.y()));
      bmax.setY(std::max(bmax.z(), bmax_part.z()));
    };


    auto const& n = blaze::normalize(blaze::cross(m_u, m_v));

    auto const o_0 = m_p + (n * m_thickness * 0.5);
    auto const u_0 = m_u;
    auto const v_0 = m_v;

    auto const o_1 = o_0 + u_0 - (n * m_thickness);
    auto const u_1 = -u_0;
    auto const v_1 = v_0;

    auto const o_2 = o_0 + v_0 - (n * m_thickness);
    auto const u_2 = n * m_thickness;
    auto const v_2 = u_0;

    auto const o_3 = o_0 + u_0 - (n * m_thickness);
    auto const u_3 = n * m_thickness;
    auto const v_3 = -u_0;

    auto const o_4 = o_0 - (n * m_thickness);
    auto const u_4 = (n * m_thickness);
    auto const v_4 = v_0;

    auto const o_5 = o_0 + u_0;
    auto const u_5 = -(n * m_thickness);
    auto const v_5 = v_0;

    appendPlaneVertexData(o_0, u_0, v_0);
    appendPlaneVertexData(o_1, u_1, v_1);
    appendPlaneVertexData(o_2, u_2, v_2);
    appendPlaneVertexData(o_3, u_3, v_3);
    appendPlaneVertexData(o_4, u_4, v_4);
    appendPlaneVertexData(o_5, u_5, v_5);


    clear();

    setVertexData(vertex_data);

    gmqt::detail::setPTeNTaBQQuick3DGeomertryAttributes(*this);
    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Triangles);
    setStride(gmqt::detail::surfacegeometry::tpSurfaceVertexElementSize());

    setBounds(bmin, bmax);
  }
}   // namespace app
