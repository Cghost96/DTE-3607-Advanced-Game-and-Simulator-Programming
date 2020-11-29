#include "scenariomodel.h"
#include "scenariowrapper.h"

// qt
#include <QColor>
#include <QMatrix4x4>

// stl
#include <numeric>
#include <memory>
#include <chrono>
using namespace std::chrono_literals;


namespace app
{



  void ScenarioModel::resetScenario(std::weak_ptr<ScenarioWrapperBase> scenario)
  {
    beginResetModel();
    m_scenario = scenario;
    endResetModel();
  }

  ScenarioModel::GeometryType
  ScenarioModel::geometryType(const QModelIndex& index) const
  {
    auto sim = m_scenario.lock();
    if (not sim) return GeometryType::NA;

    if (std::cmp_less(index.row(), sim->numberOfSpheres()))
      return GeometryType::Sphere;
    else
      return GeometryType::Plane;
  }

  int ScenarioModel::rowCount(const QModelIndex& /*index*/) const
  {
    auto sim = m_scenario.lock();
    if (not sim) return 0;

    auto no_objs = sim->numberOfSpheres() + sim->numberOfPlanes();
    return int(no_objs);
  }



  QVariant ScenarioModel::data(const QModelIndex& index, int role_in) const
  {

    if (not index.isValid()) return {};

    auto sim = m_scenario.lock();
    if (not sim) return {};

    auto const role = Roles(role_in);
    if (role == Roles::GeometryType) {
      if (geometryType(index) == GeometryType::Sphere)
        return QVariant(int(GeometryType::Sphere));
      else if (geometryType(index) == GeometryType::Plane)
        return QVariant(int(GeometryType::Plane));
      else
        return QVariant(int(GeometryType::NA));
    }
    else {
      if (geometryType(index) == GeometryType::Sphere)
        return sphereData(index, role_in);
      else if (geometryType(index) == GeometryType::Plane)
        return planeData(index, role_in);
    }


    return {};
  }



  QVariant ScenarioModel::sphereData(const QModelIndex& index,
                                     int                role_in) const
  {
    if (not index.isValid()) return {};

    auto sim = m_scenario.lock();
    if (not sim) return {};


    auto const& sphere_data_opt = sim->sphereData(size_t(index.row()));
    if (not sphere_data_opt) return {};

    auto const& sphere_data = sphere_data_opt.value();
    auto const& fo          = std::get<0>(sphere_data);
    auto const& v           = std::get<1>(sphere_data);
    auto const& r           = std::get<2>(sphere_data);

    auto fo_qt = QVector3D(float(fo[0]), float(fo[1]), float(fo[2]));
    auto v_qt  = QVector3D(float(v[0]), float(v[1]), float(v[2]));

    auto const role = Roles(role_in);

    if (role == Roles::ObjectName)
      return {(std::string("sphere <") + std::to_string(index.row())
               + std::string(">"))
                .c_str()};
    else if (role == Roles::FrameOrigin)
      return QVariant(fo_qt);
    else if (role == Roles::Velocity)
      return QVariant(v_qt);
    else if (role == Roles::RandomColor)
      return QVariant(QColor("red"));


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

  QVariant ScenarioModel::planeData(const QModelIndex& index, int role_in) const
  {
    if (not index.isValid()) return {};

    auto sim = m_scenario.lock();
    if (not sim) return {};

    const auto plane_idx = size_t(index.row()) - sim->numberOfSpheres();

    auto const& plane_data_opt = sim->fixedPlaneData(plane_idx);
    if (not plane_data_opt) return {};

    auto const& plane_data = plane_data_opt.value();
    auto const& fo         = std::get<0>(plane_data);
    auto const& n          = std::get<1>(plane_data);

    auto fo_qt = QVector3D(float(fo[0]), float(fo[1]), float(fo[2]));

    auto const role = Roles(role_in);

    if (role == Roles::ObjectName)
      return {(std::string("fixed plane <") + std::to_string(plane_idx)
               + std::string(">"))
                .c_str()};
    else if (role == Roles::FrameOrigin)
      return QVariant(fo_qt);
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

  QHash<int, QByteArray> ScenarioModel::roleNames() const
  {
    QHash<int, QByteArray> names;

    names[int(Roles::ObjectName)]  = "object_name";
    names[int(Roles::RandomColor)] = "random_color";

    names[int(Roles::FrameOrigin)] = "frame_origin";
    names[int(Roles::Velocity)]    = "velocity";

    names[int(Roles::GeometrySource)] = "geometry_source";
    names[int(Roles::Geometry)]       = "geometry_gmlib";
    names[int(Roles::GeometryType)]   = "geometry_type";

    return names;
  }

  int ScenarioModel::simLoopTimestep() const { return m_timestep.curr(); }

  int ScenarioModel::simLoopSimtime() const { return m_simtime.curr(); }

  int ScenarioModel::simLoopTimestepAverage() const { return m_timestep.avg(); }

  int ScenarioModel::simLoopSimtimeAverage() const { return m_simtime.avg(); }

  int ScenarioModel::noObjects() const { return m_no_objects_cache; }


  void ScenarioModel::updateSimulatinFrameTimings(int timestep, int simtime)
  {
    m_timestep.add(timestep);
    m_simtime.add(simtime);

    emit simLoopTimestepChanged(m_timestep.curr());
    emit simLoopSimtimeChanged(m_simtime.curr());
    emit simLoopTimestepAverageChanged(m_timestep.avg());
    emit simLoopSimtimeAverageChanged(m_simtime.avg());
  }

  void ScenarioModel::updateScene()
  {
    std::scoped_lock lock(m_modelupdate_mutex);

    auto scenario = m_scenario.lock();
    if (not scenario) return;

    const auto no_objs
      = scenario->numberOfSpheres() + scenario->numberOfPlanes();

    if (std::cmp_not_equal(m_no_objects_cache, no_objs)) {
      m_no_objects_cache = int(no_objs);
      emit noObjectsChanged(m_no_objects_cache);
    }

    if (std::cmp_less_equal(no_objs, 0)) return;

    QVector<int> updated_roles;
    updated_roles.append(int(Roles::FrameOrigin));
    updated_roles.append(int(Roles::Velocity));

    emit dataChanged(createIndex(0, 0), createIndex(int(no_objs) - 1, 0),
                     updated_roles);
  }
}   // namespace app
