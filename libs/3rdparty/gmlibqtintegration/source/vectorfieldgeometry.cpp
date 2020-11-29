#include <gmlibqtintegration/geometry/vectorfieldgeometry.h>



namespace gmqt {


  namespace detail {
    auto const updateMinMax =
      [](auto const& qp, QVector3D& bmin, QVector3D& bmax) {
        bmin = QVector3D{std::min(bmin.x(), qp.x()), std::min(bmin.y(), qp.y()),
                         std::min(bmin.z(), qp.z())};
        bmax = QVector3D{std::max(bmax.x(), qp.x()), std::max(bmax.y(), qp.y()),
                         std::max(bmax.z(), qp.z())};
      };
  }



  VectorFieldGeometry::VectorFieldGeometry(QQuick3DObject* parent)
    : CustomQuick3DGeometry(parent)
  {
  }

  float VectorFieldGeometry::radius() const { return m_radius; }

  void VectorFieldGeometry::setRadius(float radius)
  {
    m_radius = radius;
    emit radiusChanged(radius);
    updateAndMarkDirty();
  }

  CurveGeometry* VectorFieldGeometry::curveGeometry() const
  {
    return m_curve_geometry;
  }

  void VectorFieldGeometry::setCurveGeometry(CurveGeometry *curve_geometry)
  {
    resetInputVfType();
    m_curve_geometry = curve_geometry;
    emit curveGeometryChanged(curve_geometry);
    updateAndMarkDirty();
  }

  TensorProductSurfaceGeometry *VectorFieldGeometry::tpSurfaceGeometry() const
  {
    return m_tpsurface_geometry;
  }

  void VectorFieldGeometry::setTpSurfaceGeometry(
    TensorProductSurfaceGeometry* tpsurf_geometry)
  {
    resetInputVfType();
    m_tpsurface_geometry = tpsurf_geometry;
    emit tpSurfaceGeometryChanged(tpsurf_geometry);
    updateAndMarkDirty();
  }

  PolygonalSurfaceGeometry *VectorFieldGeometry::polygonalSurfaceGeometry() const
  {
    return m_psurface_geometry;
  }

  void VectorFieldGeometry::setPolygonalSurfaceGeometry(
    PolygonalSurfaceGeometry* polysurf_geometry)
  {
    resetInputVfType();
    m_psurface_geometry = polysurf_geometry;
    emit polygonalSurfaceGeometryChanged(polysurf_geometry);
    updateAndMarkDirty();
  }

  void VectorFieldGeometry::resetInputVfType() {
    m_curve_geometry     = nullptr;
    m_tpsurface_geometry = nullptr;
    m_psurface_geometry  = nullptr;
  }

  void VectorFieldGeometry::rebuildGeometry()
  {
    if(m_curve_geometry and m_curve_geometry->internalCurve())
      updateGeometryData(buildCurveVertexData());
    else if(m_tpsurface_geometry and m_tpsurface_geometry->internalSurface())
      updateGeometryData(buildTpSurfaceVertexData());
    else if (m_psurface_geometry and m_psurface_geometry->internalSurface())
      updateGeometryData(buildPolygonalSurfaceVertexData());

    return;
  }

  void VectorFieldGeometry::updateGeometryData(
    std::tuple<QByteArray, QVector3D, QVector3D> const& geometry_data)
  {
    auto const& [vertex_data,min,max] = geometry_data;

    clear();

    setVertexData(vertex_data);

    addAttribute(QQuick3DGeometry::Attribute{
      .semantic      = QQuick3DGeometry::Attribute::PositionSemantic,
      .offset        = 0,
      .componentType = QQuick3DGeometry::Attribute::F32Type});
    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Lines);
    setStride(sizeof(VertexElement));

    setBounds(min, max);
  }

  std::tuple<QByteArray, QVector3D, QVector3D>
  VectorFieldGeometry::buildCurveVertexData() const
  {
    auto const* curve = m_curve_geometry->internalCurve();

    auto const sampling_result
      = gm::parametric::curve::sample(*curve, curve->domain().I, 10ul, 1ul);

    QByteArray vertex_data;
    vertex_data.resize(int(sampling_result.size() * sizeof(LineElement)));
    auto* leptr = reinterpret_cast<LineElement*>(vertex_data.data());

    QVector3D bmin, bmax;
    {

      auto const& pos_first = sampling_result[0ul][0ul];
      bmin = bmax = utils::toQVector3D(pos_first);
    }

    for( auto const& p : sampling_result ) {

      auto const& p0 = p[0ul];
      auto const  p1 = blaze::evaluate(p0 + p[1ul]);

      auto& le       = *leptr;
      le.p0.position = utils::toQVectorND(p0);
      le.p1.position = utils::toQVectorND(p1);

      detail::updateMinMax(le.p0.position, bmin, bmax);
      detail::updateMinMax(le.p1.position, bmin, bmax);

//      qDebug() << "Vertex data; p0: " << le.p0.position
//               << ", p1: " << le.p1.position;

      leptr++;
    }

    return {vertex_data, bmin, bmax};
  }

  std::tuple<QByteArray, QVector3D, QVector3D>
  VectorFieldGeometry::buildTpSurfaceVertexData() const
  {
    auto const* surface = m_tpsurface_geometry->internalSurface();

    auto sampling_result = gm::parametric::tensorproductsurface::sample(
      *surface, {surface->domain().U.start(), surface->domain().V.start()},
      {surface->domain().U.end(), surface->domain().V.end()},
      {size_t(100), size_t(100)}, {1ul, 1ul});

    QByteArray vertex_data;
    vertex_data.resize(int(sampling_result.rows() * sampling_result.columns()
                           * sizeof(LineElement)));
    auto* leptr = reinterpret_cast<LineElement*>(vertex_data.data());

    QVector3D bmin, bmax;
    {

      auto const& pos_first = sampling_result(0ul,0ul)(0ul,0ul);
      bmin = bmax = utils::toQVector3D(pos_first);
    }

    for( auto r = 0ul; r < sampling_result.rows(); ++r ) {
      for( auto c = 0ul; c < sampling_result.columns(); ++c ) {

        auto const& p = sampling_result(r,c);

        auto const& p0 = p(0ul,0ul);
        auto const& u  = p(1ul, 0ul);
        auto const& v  = p(0ul, 1ul);
        auto const  n  = blaze::cross(blaze::subvector<0ul, 3ul>(u),
                                    blaze::subvector<0ul, 3ul>(v))
                       * 0.1;

        auto p1 = p0;
        blaze::subvector<0ul,3ul>(p1) += n;


        auto& le       = *leptr;
        le.p0.position = utils::toQVectorND(p0);
        le.p1.position = utils::toQVectorND(p1);

        detail::updateMinMax(le.p0.position, bmin, bmax);
        detail::updateMinMax(le.p1.position, bmin, bmax);

//        qDebug() << "Vertex data; p0: " << le.p0.position
//                 << ", p1: " << le.p1.position;

        leptr++;

      }
    }

    return {vertex_data, bmin, bmax};
  }

  std::tuple<QByteArray, QVector3D, QVector3D>
  VectorFieldGeometry::buildPolygonalSurfaceVertexData() const
  {
    auto const* polygon = m_psurface_geometry->internalSurface();
    using PolygonalSurfaceType = std::decay_t<decltype(*polygon)>;


    auto sampling_positions = gm::parametric::polygonsurface::
      convex_algorithms::generateTriSamplingPSpacePositions(
        size_t(10), polygon->domain().polygon);

    using DVector = typename PolygonalSurfaceType::DVector;
    auto sampling_result = gm::parametric::polygonsurface::sample(
      *polygon, sampling_positions, {DVector{1, 0}, DVector{0, 1}});

    QByteArray vertex_data;
    vertex_data.resize(int(sampling_result.size() * sizeof(LineElement)));
    auto* leptr = reinterpret_cast<LineElement*>(vertex_data.data());

    QVector3D bmin, bmax;
    {

      auto const& pos_first = sampling_result[0ul][0ul];
      bmin = bmax = utils::toQVector3D(pos_first);
    }

    for( auto const& p : sampling_result ) {

      auto const& p0 = p[0ul];
//      auto const  p1 = blaze::evaluate(p0 + p[1ul]);
      auto const& u = p[1ul];
      auto const& v = p[2ul];
      auto const  n = blaze::cross(blaze::subvector<0ul, 3ul>(u),
                                  blaze::subvector<0ul, 3ul>(v))
                     * 0.1;

      auto p1 = p0;
      blaze::subvector<0ul, 3ul>(p1) += n;

      auto& le       = *leptr;
      le.p0.position = utils::toQVectorND(p0);
      le.p1.position = utils::toQVectorND(p1);

      detail::updateMinMax(le.p0.position, bmin, bmax);
      detail::updateMinMax(le.p1.position, bmin, bmax);

//      qDebug() << "Vertex data; p0: " << le.p0.position
//               << ", p1: " << le.p1.position;

      leptr++;
    }

    return {vertex_data, bmin, bmax};

  }


}   // namespace gmqt
