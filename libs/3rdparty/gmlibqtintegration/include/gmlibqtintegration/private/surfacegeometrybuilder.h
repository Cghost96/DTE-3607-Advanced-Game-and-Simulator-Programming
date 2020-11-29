#ifndef GMLIBQTINTEGRATION_PRIVATE_SURFACEGEOMETRYBUILDER_H
#define GMLIBQTINTEGRATION_PRIVATE_SURFACEGEOMETRYBUILDER_H

#include "../private/cpp_helpers.h"

// gmlib2
#include <gmlib/core/datastructures.h>

// qt
#include <QByteArray>
#include <QQuick3DGeometry>
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>

// stl
#include <cassert>

namespace gmqt::detail {

    struct VertexElementP {
      QVector4D position;
      static constexpr auto offset = 0;
    };

    struct VertexElementTe {
      QVector2D texcoord;
      static constexpr auto offset = 0;
    };

    struct VertexElementN {
      QVector4D normal;
      static constexpr auto offset = 0;
    };

    struct VertexElementTa {
      QVector4D tangent;
      static constexpr auto offset = 0;
    };

    struct VertexElementB {
      QVector4D binormal;
      static constexpr auto offset = 0;
    };


    ///
    // Point, Texcoord, Normal, Tangent, BiNormal chain.
    struct VertexElementPTe : VertexElementP, VertexElementTe {
      static constexpr auto offset = sizeof(VertexElementP);
    };

    struct VertexElementPTeN : VertexElementPTe, VertexElementN {
      static constexpr auto offset = sizeof(VertexElementPTe);
    };

    struct VertexElementPTeNTa : VertexElementPTeN, VertexElementTa {
      static constexpr auto offset = sizeof(VertexElementPTeN);
    };

    struct VertexElementPTeNTaB : VertexElementPTeNTa, VertexElementB {
      static constexpr auto offset = sizeof(VertexElementPTeNTa);
    };

    inline void setPTeNTaBQQuick3DGeomertryAttributes(QQuick3DGeometry& geometry)
    {
      geometry.addAttribute(QQuick3DGeometry::Attribute{
        .semantic      = QQuick3DGeometry::Attribute::PositionSemantic,
        .offset        = VertexElementP::offset,
        .componentType = QQuick3DGeometry::Attribute::F32Type});

      geometry.addAttribute(QQuick3DGeometry::Attribute{
        .semantic      = QQuick3DGeometry::Attribute::TexCoordSemantic,
        .offset        = VertexElementPTe::offset,
        .componentType = QQuick3DGeometry::Attribute::F32Type});

      geometry.addAttribute(QQuick3DGeometry::Attribute{
        .semantic      = QQuick3DGeometry::Attribute::NormalSemantic,
        .offset        = VertexElementPTeN::offset,
        .componentType = QQuick3DGeometry::Attribute::F32Type});

      geometry.addAttribute(QQuick3DGeometry::Attribute{
        .semantic      = QQuick3DGeometry::Attribute::TangentSemantic,
        .offset        = sizeof(VertexElementPTeNTa::offset),
        .componentType = QQuick3DGeometry::Attribute::F32Type});

      geometry.addAttribute(QQuick3DGeometry::Attribute{
        .semantic      = QQuick3DGeometry::Attribute::BinormalSemantic,
        .offset        = sizeof(VertexElementPTeNTaB::offset),
        .componentType = QQuick3DGeometry::Attribute::F32Type});
    }


    ///
    // Point, Normal, Tangent, BiNormal chain.
    struct VertexElementPN : VertexElementP, VertexElementN {
      static constexpr auto offset = sizeof(VertexElementP);
    };

    struct VertexElementPNTa : VertexElementPN, VertexElementTa {
      static constexpr auto offset = sizeof(VertexElementPN);
    };

    struct VertexElementPNTaB : VertexElementPNTa, VertexElementB {
      static constexpr auto offset = sizeof(VertexElementPNTa);
    };

    inline void setPNBQQuick3DGeomertryAttributes(QQuick3DGeometry& geometry)
    {
      geometry.addAttribute(QQuick3DGeometry::Attribute{
        .semantic      = QQuick3DGeometry::Attribute::PositionSemantic,
        .offset        = VertexElementP::offset,
        .componentType = QQuick3DGeometry::Attribute::F32Type});

      geometry.addAttribute(QQuick3DGeometry::Attribute{
        .semantic      = QQuick3DGeometry::Attribute::NormalSemantic,
        .offset        = VertexElementPN::offset,
        .componentType = QQuick3DGeometry::Attribute::F32Type});

      geometry.addAttribute(QQuick3DGeometry::Attribute{
        .semantic      = QQuick3DGeometry::Attribute::TangentSemantic,
        .offset        = sizeof(VertexElementPNTa::offset),
        .componentType = QQuick3DGeometry::Attribute::F32Type});

      geometry.addAttribute(QQuick3DGeometry::Attribute{
        .semantic      = QQuick3DGeometry::Attribute::BinormalSemantic,
        .offset        = sizeof(VertexElementPNTaB::offset),
        .componentType = QQuick3DGeometry::Attribute::F32Type});
    }



  namespace surfacegeometry {

    using TPSurfaceSamplingResult = gm::datastructures::parametrics::
      tensorproductsurface::SamplingResult<gm::VectorT<double, 4>>;

    using TPSVertexElement = VertexElementPTeNTaB;


    inline int tpSurfaceVertexElementSize()
    {
      return sizeof(TPSVertexElement);
    }

    inline int tpSurfaceNoTriFaces(int no_samples, int no_slices)
    {
      auto const no_internal_quad_faces   = (no_samples - 1) * (no_slices - 1);
      auto constexpr no_trifaces_per_quad = 2;
      return no_internal_quad_faces * no_trifaces_per_quad;
    }

    inline int tpSurfaceNoTriFaceVerticesWithDuplicates(int no_samples,
                                                        int no_slices)
    {
      auto constexpr no_vertices_per_face = 3;
      return tpSurfaceNoTriFaces(no_samples, no_slices) * no_vertices_per_face;
    }

    //    struct IndexElement {
    //      quint32 i0;
    //      quint32 i1;
    //      quint32 i2;
    //    };


    inline auto buildTPSurfaceGeometryStandaloneVertexBuffer(
      TPSurfaceSamplingResult const& sampling_result)
    {

      auto fillVertexElement =
        [sampling_result](TPSVertexElement& ve, int r, int c) {
          auto const& samp = sampling_result(size_t(r), size_t(c));
          auto const  p    = blaze::evaluate(samp(0, 0));
//          std::cout << "fillVertexElement (r,c) : (" << r << ',' << c
//                    << ") -- p: " << p[0] << ',' << p[1] << ',' << p[2] << ','
//                    << p[3] << std::endl;
          ve.position = utils::toQVectorND(p);

          // Tex coord of sample
          auto const tu = float(r) / float(int(sampling_result.rows()) - 1);
          auto const tv = float(c) / float(int(sampling_result.columns()) - 1);
          ve.texcoord   = {tu, tv};

          // Normal of sample
          auto const& u = samp(1, 0);
          auto const& v = samp(0, 1);
          auto const  N = blaze::normalize(
            blaze::cross(blaze::subvector<0, 3>(u), blaze::subvector<0, 3>(v)));
          ve.normal
            = utils::toQVector4D<utils::VecType::Vector>(blaze::evaluate(N));

          // Tangent of sample
          ve.tangent = utils::toQVectorND(u);

          // Bi-Normal of sample
          ve.binormal = utils::toQVectorND(v);
        };


      QByteArray vertex_data;

      auto const MR = int(sampling_result.rows());
      auto const MC = int(sampling_result.columns());

      //      auto const no_vertices = int(m_no_samples * m_no_slices);
//      auto const no_faces = tpSurfaceNoTriFaces(MR, MC);
//      auto const stride   = tpSurfaceVertexElementSize();

//      std::cout << "Rebuild geometry -- curve geometry" << std::endl;
//      std::cout << " name: " << name().toStdString() << std::endl;
//      std::cout << " no samples: " << MR << std::endl;
//      std::cout << " no slices: " << MC << std::endl;
      //      std::cout << " no vertices: " << no_vertices << std::endl;
//      std::cout << " no_faces: " << no_faces << std::endl;
      //    std::cout << " curve s/e: " << curve.intervalStart() << "/"
      //              << curve.intervalEnd() << std::endl;
//      std::cout << " stride: " << stride << std::endl;



      vertex_data.resize(tpSurfaceNoTriFaceVerticesWithDuplicates(MR, MC)
                         * tpSurfaceVertexElementSize());

//      std::cout << " --" << std::endl;
//      std::cout << " Vd size: " << vertex_data.size() << std::endl;



      // Construction Vertex Data
      TPSVertexElement* veptr
        = reinterpret_cast<TPSVertexElement*>(vertex_data.data());
      for (auto r = 0; r < MR - 1; ++r) {
        for (auto c = 0; c < MC - 1; ++c) {
          // Tri 1
          fillVertexElement(*veptr++, r, c);
          fillVertexElement(*veptr++, r + 1, c + 1);
          fillVertexElement(*veptr++, r, c + 1);

          // Tri 1
          fillVertexElement(*veptr++, r, c);
          fillVertexElement(*veptr++, r + 1, c);
          fillVertexElement(*veptr++, r + 1, c + 1);
        }
      }


      // bounds min/max
      QVector3D bmin, bmax;
      {
        auto const& samp = sampling_result(0, 0);
        auto const& p    = samp(0, 0);

        bmin = bmax = utils::toQVector3D(p);
      }

      for (auto r = 0; r < MR; ++r) {
        for (auto c = 1; c < MC; ++c) {

          auto const& samp = sampling_result(size_t(r), size_t(c));
          auto const& p    = samp(0, 0);
          auto const& qp   = utils::toQVector3D(p);

          bmin
            = QVector3D{std::min(bmin.x(), qp.x()), std::min(bmin.y(), qp.y()),
                        std::min(bmin.z(), qp.z())};
          bmax
            = QVector3D{std::max(bmax.x(), qp.x()), std::max(bmax.y(), qp.y()),
                        std::max(bmax.z(), qp.z())};
        }
      }

      return std::tuple(vertex_data, bmin, bmax);
    }



    using PolygonalSurfaceSamplingResult = gm::datastructures::parametrics::
      polygonalsurface::SamplingResultDVec<gm::VectorT<double, 4>>;
    using PolygonalSurfaceSamplingResultFaceIndices
      = std::vector<std::array<size_t, 3>>;

    using PSVertexElement = VertexElementPNTaB;

    inline int polygonSurfaceVertexElementSize()
    {
      return sizeof(PSVertexElement);
    }





    inline auto buildPolygonalSurfaceGeometryStandaloneVertexBuffer(
      PolygonalSurfaceSamplingResult const&            sampling_result,
      PolygonalSurfaceSamplingResultFaceIndices const& face_indices)
    {
      auto fillVertexElement =
        [sampling_result](PSVertexElement& ve, int idx) {
          auto const& samp = sampling_result[size_t(idx)];
          auto const& p    = samp[0];
          ve.position      = utils::toQVectorND(p);

          // Tex coord of sample
          //          auto const tu = float(r) /
          //          float(int(sampling_result.rows()) - 1); auto const tv =
          //          float(c) / float(int(sampling_result.columns()) - 1);
          //          ve.texcoord   = {tu, tv};
//          ve.texcoord   = {0,0};

          // Normal of sample
          auto const& u = samp[1];
          auto const& v = samp[2];
          auto const  N = blaze::normalize(
            blaze::cross(blaze::subvector<0, 3>(u), blaze::subvector<0, 3>(v)));
          ve.normal
            = utils::toQVector4D<utils::VecType::Vector>(blaze::evaluate(N));

          // Tangent of sample
          ve.tangent = utils::toQVectorND(u);

          // Bi-Normal of sample
          ve.binormal = utils::toQVectorND(v);
        };


      QByteArray vertex_data;
      auto const no_faces = int(face_indices.size());
//      auto const stride   = sizeof(PSVertexElement);

//      std::cout << "Rebuild geometry -- curve geometry" << std::endl;
//      std::cout << " name: " << name().toStdString() << std::endl;
//      std::cout << " no samples: " << MR << std::endl;
//      std::cout << " no slices: " << MC << std::endl;
      //      std::cout << " no vertices: " << no_vertices << std::endl;
//      std::cout << " no_faces: " << no_faces << std::endl;
      //    std::cout << " curve s/e: " << curve.intervalStart() << "/"
      //              << curve.intervalEnd() << std::endl;
//      std::cout << " stride: " << stride << std::endl;



      vertex_data.resize(3 * no_faces * polygonSurfaceVertexElementSize());

//      std::cout << " --" << std::endl;
//      std::cout << " Vd size: " << vertex_data.size() << std::endl;



      // Construction Vertex Data
      PSVertexElement* veptr
        = reinterpret_cast<PSVertexElement*>(vertex_data.data());
      for (auto i = 0ul; i < face_indices.size(); ++i) {
        // Tri
        fillVertexElement(*veptr++, int(face_indices[i][0]));
        fillVertexElement(*veptr++, int(face_indices[i][1]));
        fillVertexElement(*veptr++, int(face_indices[i][2]));
      }


      // bounds min/max
      QVector3D bmin, bmax;
      {
        auto const& samp = sampling_result[0];
        auto const& p    = samp[0];

        bmin = bmax = utils::toQVector3D(p);
      }

      for (auto i = 1ul; i < sampling_result.size(); ++i) {

        auto const& samp = sampling_result[i];
        auto const& p    = samp[0];
        auto const& qp   = utils::toQVector3D(p);

        bmin = QVector3D{std::min(bmin.x(), qp.x()), std::min(bmin.y(), qp.y()),
                         std::min(bmin.z(), qp.z())};
        bmax = QVector3D{std::max(bmax.x(), qp.x()), std::max(bmax.y(), qp.y()),
                         std::max(bmax.z(), qp.z())};
      }

      return std::tuple(vertex_data, bmin, bmax);
    }



  }   // namespace surfacegeometry


}   // namespace gmqt

#endif // GMLIBQTINTEGRATION_PRIVATE_SURFACEGEOMETRYBUILDER_H
