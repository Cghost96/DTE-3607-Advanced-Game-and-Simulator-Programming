#ifndef GMLIBQTINTEGRATION_PRIVATE_CUSTOMQUICK3DGEOMETRY_H
#define GMLIBQTINTEGRATION_PRIVATE_CUSTOMQUICK3DGEOMETRY_H


// integration
#include "../private/cpp_helpers.h"

// qt
#include <QQuick3DGeometry>
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>
#include <QSize>

// stl
#include <string>

namespace gmqt
{


  class CustomQuick3DGeometry : public QQuick3DGeometry {
    Q_OBJECT

  public:
    CustomQuick3DGeometry(QQuick3DObject* parent = nullptr);

  protected:
    bool m_dirty{true};

    void updateAndMarkDirty();

    QSSGRenderGraphObject*
    updateSpatialNode(QSSGRenderGraphObject* node) override;

    virtual void rebuildGeometry() = 0;


  public:
    static auto constexpr qmlPkgName{"GMlib2.Geometry"};
    static auto constexpr qmlPkgVersionMajor{0};
    static auto constexpr qmlPkgVersionMinor{1};

    template <typename GeometryType_T>
    static void registerQmlType(const char* qml_name);

    template <typename GeometryType_T>
    static void registerUncreatableQmlType(const char* qml_name);

    template <typename... Ts>
    void setCustomGeneratedName(Ts const& ... ts);
  };



  template <typename GeometryType_T>
  void CustomQuick3DGeometry::registerQmlType(const char* qml_name)
  {
    qmlRegisterType<GeometryType_T>(CustomQuick3DGeometry::qmlPkgName,
                                    CustomQuick3DGeometry::qmlPkgVersionMajor,
                                    CustomQuick3DGeometry::qmlPkgVersionMinor,
                                    qml_name);
  }

  template <typename GeometryType_T>
  void CustomQuick3DGeometry::registerUncreatableQmlType(const char* qml_name)
  {
    qmlRegisterUncreatableType<GeometryType_T>(
      CustomQuick3DGeometry::qmlPkgName,
      CustomQuick3DGeometry::qmlPkgVersionMajor,
      CustomQuick3DGeometry::qmlPkgVersionMinor, qml_name, "Super Class !!");
  }

  template <typename... Ts>
  void CustomQuick3DGeometry::setCustomGeneratedName(Ts const& ... ts)
  {
    QString name("#gmlib_curve_geometry");

    auto name_gen =
      [&name]<typename LT, typename... LTs>(
          auto& name_gen_ref, LT const& first, LTs const&... rest)
    {

      if constexpr(std::is_same_v<LT,QString>)
        name.append("_" + first);
      else if constexpr (std::is_same_v<LT,float>)
        name.append(QString("_%1").arg(double(first)));
      else
        name.append(QString("_%1").arg(QVariant(first).toString()));

      if constexpr (sizeof...(LTs) > 0)
        return name_gen_ref(name_gen_ref,rest...);
      else return name;
    };

    setName(name_gen(name_gen,ts...));
  }



}   // namespace gmqt



#endif   // GMLIBQTINTEGRATION_PRIVATE_CUSTOMQUICK3DGEOMETRY_H
