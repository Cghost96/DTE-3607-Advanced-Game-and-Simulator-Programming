#ifndef LIMITEDPLANEGEOMETRY_H
#define LIMITEDPLANEGEOMETRY_H


// GMlib qt integration
#include <gmlibqtintegration/private/customquick3dgeometry.h>


// gmlib
#include "gmlib/parametric/classic_objects/plane.h"


namespace app
{

  class LimitedPlaneGeometry : public gmqt::CustomQuick3DGeometry {
    Q_OBJECT

  public:
    LimitedPlaneGeometry(QQuick3DObject* parent = nullptr);


    using EmbedSpaceType            = gm::spaces::ProjectiveSpace<double, 3ul>;
    using Point                     = EmbedSpaceType::Point;
    using Vector                    = EmbedSpaceType::Vector;
    using VectorH                   = EmbedSpaceType::VectorH;
    using Type                      = EmbedSpaceType::Type;
    static constexpr auto VectorDim = EmbedSpaceType::VectorDim;

    void set(Point const& p, Vector const& u, Vector const& v,
             Type const& thickness);

    void sample();

  protected:
    void rebuildGeometry() override final;



    using PlaneType = gm::parametric::Plane<EmbedSpaceType>;



    EmbedSpaceType::Point  m_p{0, 0, 0};
    EmbedSpaceType::Vector m_u{1, 0, 0};
    EmbedSpaceType::Vector m_v{0, 0, 1};
    EmbedSpaceType::Type   m_thickness{0.01};
  };

}   // namespace gmqt


#endif   // LIMITEDPLANEGEOMETRY_H
