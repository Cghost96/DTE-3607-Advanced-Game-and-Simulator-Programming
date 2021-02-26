#include "scenariomodel.h"
#include "scenariowrapper.h"

#include "limitedplanegeometry.h"

// gmlib qt integration
#include <gmlibqtintegration/geometry/surfaces/tensorproduct/planegeometry.h>
#include <gmlibqtintegration/geometry/surfaces/tensorproduct/spheregeometry.h>
#include <gmlibqtintegration/geometry/surfaces/tensorproduct/beziersurfacegeometry.h>

// qt
#include <QColor>
#include <QMatrix4x4>

// stl
#include <numeric>
#include <memory>
#include <chrono>
using namespace std::chrono_literals;




namespace app {

  namespace d {

    inline QMatrix3x3 toQtRotationFrame(app::Frame3H const& frame_h) {
      QMatrix3x3  rf_qt;
      auto const& rf = blaze::submatrix<0ul, 0ul, 3ul, 3ul>(frame_h);

      rf_qt(0, 0) = float(rf(0, 0));
      rf_qt(0, 1) = float(rf(0, 1));
      rf_qt(0, 2) = float(rf(0, 2));
      rf_qt(1, 0) = float(rf(1, 0));
      rf_qt(1, 1) = float(rf(1, 1));
      rf_qt(1, 2) = float(rf(1, 2));
      rf_qt(2, 0) = float(rf(2, 0));
      rf_qt(2, 1) = float(rf(2, 1));
      rf_qt(2, 2) = float(rf(2, 2));

      return rf_qt;
    }

    inline QVector3D toQtFrameOrigin(app::Frame3H const& frame_h) {
      QVector3D   origin;
      auto const& fo = blaze::subvector<0ul, 3ul>(blaze::column<3ul>(frame_h));
      origin[0]      = float(fo[0]);
      origin[1]      = float(fo[1]);
      origin[2]      = float(fo[2]);
      return origin;
    }

    inline QVector3D toQtVector3D(app::Vector3 const& p_v) {

      return QVector3D{float(p_v[0]), float(p_v[1]), float(p_v[2])};
    }


  }   // namespace d


  void ScenarioModel::resetScenario(std::weak_ptr<ScenarioWrapperBase> scenario) {
    beginResetModel();
    m_scenario = scenario;
    endResetModel();
  }

  ScenarioModel::GeometryType ScenarioModel::geometryType(const QModelIndex& index) const {
    auto sim = m_scenario.lock();
    if (not sim) return GeometryType::NA;

    if (index.row() < 0) return GeometryType::NA;

    return GeometryType(sim->geometryType(size_t(index.row())));
  }

  int ScenarioModel::rowCount(const QModelIndex& /*index*/) const {
    auto sim = m_scenario.lock();
    if (not sim) return 0;

    auto no_objs = sim->totalNumberOfObjects();
    return int(no_objs);
  }



  QVariant ScenarioModel::data(const QModelIndex& index, int role_in) const {

    if (not index.isValid()) return {};

    auto sim = m_scenario.lock();
    if (not sim) return {};

    auto const role = Roles(role_in);
    if (role == Roles::GeometryType) {
      if (geometryType(index) == GeometryType::Sphere)
        return QVariant(int(GeometryType::Sphere));
      else if (geometryType(index) == GeometryType::Plane)
        return QVariant(int(GeometryType::Plane));
      else if (geometryType(index) == GeometryType::LimitedPlane)
        return QVariant(int(GeometryType::LimitedPlane));
      else if (geometryType(index) == GeometryType::BezierSurface)
        return QVariant(int(GeometryType::BezierSurface));
      else
        return QVariant(int(GeometryType::NA));
    }
    else {
      if (geometryType(index) == GeometryType::Sphere)
        return sphereData(index, role_in);
      else if (geometryType(index) == GeometryType::Plane)
        return planeData(index, role_in);
      else if (geometryType(index) == GeometryType::LimitedPlane)
        return limitedPlaneData(index, role_in);
      else if (geometryType(index) == GeometryType::BezierSurface)
        return bezierSurfaceData(index, role_in);
    }


    return {};
  }



  QVariant ScenarioModel::sphereData(const QModelIndex& index, int role_in) const {
    if (not index.isValid()) return {};

    auto sim = m_scenario.lock();
    if (not sim) return {};


    auto const& sphere_data_opt = sim->sphereData(size_t(index.row()));
    if (not sphere_data_opt) return {};

    auto const& sphere_data = sphere_data_opt.value();

    auto const& frame_h = std::get<0>(sphere_data);
    auto const  fo_qt   = d::toQtFrameOrigin(frame_h);
    auto const  rf_qt   = d::toQtRotationFrame(frame_h);

    auto const& v     = std::get<1>(sphere_data);
    auto const  v_qt  = d::toQtVector3D(v);
    auto const& r     = std::get<2>(sphere_data);
    auto const  ea_qt = QQuaternion::fromRotationMatrix(rf_qt).toEulerAngles();

    auto const& isFixed = std::get<3>(sphere_data);
    auto const  role    = Roles(role_in);

    if (role == Roles::ObjectName)
      return {(std::string("sphere <") + std::to_string(index.row()) + std::string(">")).c_str()};
    else if (role == Roles::FrameOrigin)
      return QVariant(fo_qt);
    else if (role == Roles::Velocity)
      return QVariant(v_qt);
    else if (role == Roles::Radius)
      return QVariant(r);
    else if (role == Roles::EulerRotation)
      return QVariant(ea_qt);
    else if (role == Roles::RandomColor)
      return QVariant(QColor("red"));
    else if (role == Roles::FixedSphere)
      return QVariant(isFixed);


    // Geometry
    else if (role == Roles::GeometrySource) {
      return QVariant("#Sphere");
    }
    else if (role == Roles::Geometry) {

      auto* sphere_sp = new gmqt::SphereGeometry;
      sphere_sp->setRadius(float(r));
      sphere_sp->setNoSamples({10, 10});
      return QVariant::fromValue(sphere_sp);
    }
    // Geometry [end]


    // Fallthrough
    return {};
  }

  QVariant ScenarioModel::planeData(const QModelIndex& index, int role_in) const {
    if (not index.isValid()) return {};

    auto sim = m_scenario.lock();
    if (not sim) return {};

    const auto plane_idx = size_t(index.row()) - sim->planesOffset();

    auto const& plane_data_opt = sim->planeData(plane_idx);
    if (not plane_data_opt) return {};

    auto const& plane_data = plane_data_opt.value();

    auto const& frame_h      = std::get<0>(plane_data);
    auto const& plane_origin = std::get<1>(plane_data);
    auto const  fo_qt        = d::toQtVector3D(plane_origin);

    auto const rf_qt = d::toQtRotationFrame(frame_h);
    auto const ea_qt = QQuaternion::fromRotationMatrix(rf_qt).toEulerAngles();

    auto const& n = std::get<2>(plane_data);


    auto const role = Roles(role_in);
    if (role == Roles::ObjectName)
      return {(std::string("fixed plane <") + std::to_string(plane_idx) + std::string(">")).c_str()};
    else if (role == Roles::FrameOrigin)
      return QVariant(fo_qt);
    else if (role == Roles::EulerRotation)
      return QVariant(ea_qt);
    else if (role == Roles::Geometry) {

      auto* plane_sp = new gmqt::PlaneGeometry;

      auto const n_liv = gm::algorithms::linearIndependentVector(n);
      auto const u     = blaze::cross(n_liv, n);
      auto const v     = blaze::cross(u, n);

      auto const us = blaze::normalize(u) * 25.;
      auto const vs = blaze::normalize(v) * 25.;

      auto const o = -((us + vs) * 0.5);



      plane_sp->setOrigin({float(o[0]), float(o[1]), float(o[2])});
      plane_sp->setUAxis({float(us[0]), float(us[1]), float(us[2])});
      plane_sp->setVAxis({float(vs[0]), float(vs[1]), float(vs[2])});
      plane_sp->setNoSamples({2, 2});

      return QVariant::fromValue(plane_sp);
    }

    // Fallthrough
    return {};
  }

  QVariant ScenarioModel::limitedPlaneData(const QModelIndex& index, int role_in) const {
    if (not index.isValid()) return {};

    auto sim = m_scenario.lock();
    if (not sim) return {};

    const auto lplane_idx = size_t(index.row()) - sim->limitedPlanesOffset();

    auto const& lplane_data_opt = sim->limitedPlaneData(lplane_idx);
    if (not lplane_data_opt) return {};

    auto const& lplane_data = lplane_data_opt.value();

    auto const& frame_h = std::get<0>(lplane_data);
    auto const& p       = std::get<1>(lplane_data);
    auto const& u       = std::get<2>(lplane_data);
    auto const& v       = std::get<3>(lplane_data);
    auto const& t       = std::get<4>(lplane_data);

    auto const fo_qt = d::toQtFrameOrigin(frame_h);

    auto const rf_qt = d::toQtRotationFrame(frame_h);
    auto const ea_qt = QQuaternion::fromRotationMatrix(rf_qt).toEulerAngles();

    auto const role = Roles(role_in);
    if (role == Roles::ObjectName)
      return {(std::string("fixed plane <") + std::to_string(lplane_idx) + std::string(">")).c_str()};
    else if (role == Roles::FrameOrigin)
      return QVariant(fo_qt);
    else if (role == Roles::EulerRotation)
      return QVariant(ea_qt);
    else if (role == Roles::Geometry) {



      LimitedPlaneGeometry* lplane = new LimitedPlaneGeometry;
      lplane->set(p, u, v, t);
      return QVariant::fromValue(lplane);
    }

    // Fallthrough
    return {};
  }

  QVariant ScenarioModel::bezierSurfaceData(const QModelIndex& index, int role_in) const {
    if (not index.isValid()) return {};

    auto sim = m_scenario.lock();
    if (not sim) return {};

    const auto bez_idx = size_t(index.row()) - sim->bezierSurfacesOffset();

    auto const& bez_data_opt = sim->bezierSurfaceData(bez_idx);
    if (not bez_data_opt) return {};

    [[maybe_unused]] auto const& bez_data = bez_data_opt.value();

    auto const& frame_h = std::get<0>(bez_data);
    auto const& cn      = std::get<1>(bez_data);

    auto const fo_qt = d::toQtFrameOrigin(frame_h);

    auto const rf_qt = d::toQtRotationFrame(frame_h);
    auto const ea_qt = QQuaternion::fromRotationMatrix(rf_qt).toEulerAngles();

    auto const role = Roles(role_in);
    if (role == Roles::ObjectName)
      return {(std::string("bezier surface <") + std::to_string(bez_idx) + std::string(">")).c_str()};
    else if (role == Roles::FrameOrigin)
      return QVariant(fo_qt);
    else if (role == Roles::EulerRotation)
      return QVariant(ea_qt);
    else if (role == Roles::Geometry) {
      using BzSG              = gmqt::BezierSurfaceGeometry;
      auto* bezsurf           = new BzSG();
      bezsurf->m_bezsurf->m_C = cn;
      bezsurf->setNoSamples({15, 15});
      return QVariant::fromValue(bezsurf);
    }

    // Fallthrough
    return {};
  }

  QHash<int, QByteArray> ScenarioModel::roleNames() const {
    QHash<int, QByteArray> names;

    names[int(Roles::ObjectName)]  = "object_name";
    names[int(Roles::RandomColor)] = "random_color";

    names[int(Roles::FrameOrigin)]   = "frame_origin";
    names[int(Roles::Velocity)]      = "velocity";
    names[int(Roles::EulerRotation)] = "euler_rotation";
    names[int(Roles::Radius)]        = "radius";

    names[int(Roles::GeometrySource)] = "geometry_source";
    names[int(Roles::Geometry)]       = "geometry_gmlib";
    names[int(Roles::GeometryType)]   = "geometry_type";

    names[int(Roles::FixedSphere)] = "fixed_sphere";

    return names;
  }

  int ScenarioModel::simLoopTimestep() const { return m_timestep.curr(); }

  int ScenarioModel::simLoopSimtime() const { return m_simtime.curr(); }

  int ScenarioModel::simLoopTimestepAverage() const { return m_timestep.avg(); }

  int ScenarioModel::simLoopSimtimeAverage() const { return m_simtime.avg(); }

  int ScenarioModel::noObjects() const { return m_no_objects_cache; }


  void ScenarioModel::updateSimulatinFrameTimings(int timestep, int simtime) {
    m_timestep.add(timestep);
    m_simtime.add(simtime);

    emit simLoopTimestepChanged(m_timestep.curr());
    emit simLoopSimtimeChanged(m_simtime.curr());
    emit simLoopTimestepAverageChanged(m_timestep.avg());
    emit simLoopSimtimeAverageChanged(m_simtime.avg());
  }

  void ScenarioModel::updateScene() {
    std::scoped_lock lock(m_modelupdate_mutex);

    auto scenario = m_scenario.lock();
    if (not scenario) return;

    const auto no_objs = scenario->numberOfSpheres() + scenario->numberOfPlanes();

    if (std::cmp_not_equal(m_no_objects_cache, no_objs)) {
      m_no_objects_cache = int(no_objs);
      emit noObjectsChanged(m_no_objects_cache);
    }

    if (std::cmp_less_equal(no_objs, 0)) return;

    QVector<int> updated_roles;
    updated_roles.append(int(Roles::FrameOrigin));
    updated_roles.append(int(Roles::EulerRotation));
    updated_roles.append(int(Roles::Velocity));

    emit dataChanged(createIndex(0, 0), createIndex(int(no_objs) - 1, 0), updated_roles);
  }
}   // namespace app
