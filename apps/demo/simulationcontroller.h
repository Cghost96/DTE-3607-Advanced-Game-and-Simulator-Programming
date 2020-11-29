#ifndef DTE3607_SIMULATOIONCONTROLLER_H
#define DTE3607_SIMULATOIONCONTROLLER_H

#include "types.h"
#include "simulationloopworker.h"
#include "scenariowrapper.h"

// qt
#include <QObject>
#include <QThread>
#include <QTimer>
#include <QDebug>

// stl
#include <chrono>
#include <mutex>

namespace app
{

  class SimulationController : public QObject {
    Q_OBJECT

    QThread simulation_loop_thread;
    QTimer  simulation_loop_timer;
    QTimer  simulation_model_update_timer;


  public:
    SimulationController() = default;
    ~SimulationController() override;

    void resetScenario(std::weak_ptr<ScenarioWrapperBase> scenario_wptr);

  private:
    void clearScenario();
    void startSimulationTimer();
    void stopSimulationTimer();

  public slots:
    void startStopSimulationTimer();

  signals:
    void startStopSimulator();
    void resetSimulator(QString scenario_name);
    void setSimulationTimestep(int timestep);

    void relayReadySimulationFrameTimings(int timestep, int simtime);

    void simulationStateReady();

    void initSimLoopWorker();
  };

}   // namespace app


#endif   // DTE3607_SIMULATOIONCONTROLLER_H
