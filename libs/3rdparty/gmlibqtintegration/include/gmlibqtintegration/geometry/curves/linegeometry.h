#ifndef GMLIBQTINTEGRATION_GEOMETRY_CURVES_LINEGEOMETRY_H
#define GMLIBQTINTEGRATION_GEOMETRY_CURVES_LINEGEOMETRY_H


#include "../curvegeometry.h"

// gmlib
#include "gmlib/parametric/classic_objects/line.h"


namespace gmqt
{

  class LineGeometry : public CurveGeometry {
    Q_OBJECT

    Q_PROPERTY(
      QVector3D origin READ origin WRITE setOrigin NOTIFY originChanged)
    Q_PROPERTY(QVector3D direction READ direction WRITE setDirection NOTIFY
                 directionChanged)

    using LineCurveType
      = gm::parametric::Line<gm::spaces::ProjectiveSpace<double, 3ul>>;

  public:
    LineGeometry(QQuick3DObject* parent = nullptr) : CurveGeometry(parent)
    {


      //      setName(utils::toQString("#gmlib_geometry_line",
      //                               origin().x(),origin().y(),origin().z(),
      //                               direction().x(),direction().y(),direction().z()));

      m_line_curve = std::make_unique<LineCurveType>(
        utils::toGMVectorT(m_origin), utils::toGMVectorT(m_direction));
      setInternalCurve(m_line_curve.get());
      updateAndMarkDirty();
    }

    QVector3D origin() const { return m_origin; }
    QVector3D direction() const { return m_direction; }

    void setOrigin(QVector3D origin)
    {
      m_origin           = origin;
      m_line_curve->m_pt = utils::toGMVectorH4D(m_origin);
      emit originChanged(origin);
      updateAndMarkDirty();
    }
    void setDirection(QVector3D direction)
    {
      m_direction       = direction;
      m_line_curve->m_v = utils::toGMVectorH4D(m_direction);
      emit directionChanged(direction);
      updateAndMarkDirty();
    }

  signals:
    void originChanged(QVector3D origin);
    void directionChanged(QVector3D direction);


  private:
    QVector3D                      m_origin{0, 0, 0};
    QVector3D                      m_direction{10, 0, 0};
    std::unique_ptr<LineCurveType> m_line_curve;
  };
}   // namespace gmqt


#endif   // GMLIBQTINTEGRATION_GEOMETRY_CURVES_LINEGEOMETRY_H
