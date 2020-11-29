#ifndef DTE3607_SIMULATIONLOOPWORKER_H
#define DTE3607_SIMULATIONLOOPWORKER_H

#include "types.h"
#include "scenariowrapper.h"

// coldet
#include <coldet/bits/types.h>

// qt
#include <QObject>
#include <QDebug>

// stl
#include <mutex>
#include <iostream>
#include <optional>
#include <chrono>


namespace app
{
  using namespace std::chrono_literals;

  class SimulationLoopWorker : public QObject {
    Q_OBJECT

  public:
    SimulationLoopWorker(std::weak_ptr<ScenarioWrapperBase> scenario,
                         NanoSeconds                        timestep);

  public slots:
    void simulate();
    void reInitialize();

  private:
    std::weak_ptr<ScenarioWrapperBase> m_scenario;
    bool                               m_initialized{false};
    std::mutex                         m_mutex;
    std::optional<TP>                  m_prev_timepoint;
    NanoSeconds                        m_timestep;

  signals:
    void initialized();
    void simulationFrameTimingsReady(int timestep, int simtime);
  };

}   // namespace app

#endif   // DTE3607_SIMULATIONLOOPWORKER_H
