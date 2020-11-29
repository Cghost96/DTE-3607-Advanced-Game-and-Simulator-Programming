#ifndef DTE3607_SCENARIOMODEL_H
#define DTE3607_SCENARIOMODEL_H


#include "types.h"
#include "scenariowrapper.h"

// coldet
#include <coldet/bits/types.h>

// gmlib qt integration
#include <gmlibqtintegration/geometry/surfaces/tensorproduct/planegeometry.h>
#include <gmlibqtintegration/geometry/surfaces/tensorproduct/spheregeometry.h>

// qt
#include <QAbstractListModel>
#include <QPointer>
#include <mutex>



namespace app
{


  class ScenarioModel : public QAbstractListModel {
    Q_OBJECT

    Q_PROPERTY(
      int simLoopTimestep READ simLoopTimestep NOTIFY simLoopTimestepChanged)
    Q_PROPERTY(
      int simLoopSimtime READ simLoopSimtime NOTIFY simLoopSimtimeChanged)

    Q_PROPERTY(int simLoopTimestepAverage READ simLoopTimestepAverage NOTIFY
                 simLoopTimestepAverageChanged)
    Q_PROPERTY(int simLoopSimtimeAverage READ simLoopSimtimeAverage NOTIFY
                 simLoopSimtimeAverageChanged)

    Q_PROPERTY(int noObjects READ noObjects NOTIFY noObjectsChanged)

  public:
    ScenarioModel() = default;

    enum class Roles {
      ObjectName = Qt::UserRole,
      RandomColor,

      GeometrySource,
      Geometry,

      GeometryType,

      FrameOrigin,
      Velocity
    };

    void resetScenario(std::weak_ptr<ScenarioWrapperBase> scenario);

    enum class GeometryType : int { NA, Sphere, Plane };
    Q_ENUM(GeometryType)

    GeometryType geometryType(const QModelIndex& index) const;

    // mandatory
    int      rowCount(const QModelIndex& index) const override;
    QVariant data(const QModelIndex& index,
                  int                role = Qt::DisplayRole) const override;

    QVariant sphereData(const QModelIndex& index,
                        int                role = Qt::DisplayRole) const;
    QVariant planeData(const QModelIndex& index,
                       int                role = Qt::DisplayRole) const;

    // optional
    QHash<int, QByteArray> roleNames() const override;

    // stuff
    int simLoopTimestep() const;
    int simLoopSimtime() const;

    int simLoopTimestepAverage() const;
    int simLoopSimtimeAverage() const;

    int noObjects() const;

  private:
    class TimingCntr {
      int m_curr{0};
      int m_total{0};
      int m_no{0};

    public:
      void add(int rhs)
      {
        m_curr = rhs;
        m_total += m_curr;
        ++m_no;
      }

      int avg() const
      {
        if (m_no == 0) return 0;

        return m_total / m_no;
      }

      int curr() const { return m_curr; }
    };

    std::weak_ptr<ScenarioWrapperBase> m_scenario;
    TimingCntr                         m_timestep;
    TimingCntr                         m_simtime;
    mutable int                        m_no_objects_cache{0};
    std::optional<TP>                  m_prev_update_request_timepoint;
    std::mutex                         m_modelupdate_mutex;

  public slots:
    void updateSimulatinFrameTimings(int timestep, int simtime);
    void updateScene();

  signals:
    void simLoopTimestepChanged(int timestep);
    void simLoopSimtimeChanged(int simtime);
    void simLoopTimestepAverageChanged(int average);
    void simLoopSimtimeAverageChanged(int average);
    void noObjectsChanged(int no_objects);
  };

}   // namespace app


#endif   // DTE3607_SCENARIOMODEL_H
