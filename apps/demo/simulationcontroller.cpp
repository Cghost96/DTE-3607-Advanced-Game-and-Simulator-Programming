#include "simulationcontroller.h"

namespace app {

  SimulationController::~SimulationController() {
    simulation_loop_thread.quit();
    simulation_loop_thread.wait();
  }

  void SimulationController::resetScenario(std::weak_ptr<ScenarioWrapperBase> scenario_wptr) {
    clearScenario();

    auto scenario = scenario_wptr.lock();
    if (not scenario) return;

    // Construct worker and move to thread
    auto* sim_loop_worker = new SimulationLoopWorker(scenario, 16ms);
    connect(&simulation_loop_thread, &QThread::finished, sim_loop_worker, &QObject::deleteLater);
    connect(sim_loop_worker, &SimulationLoopWorker::simulationFrameTimingsReady, this,
            &SimulationController::relayReadySimulationFrameTimings);
    connect(sim_loop_worker, &SimulationLoopWorker::initialized, this,
            &SimulationController::simulationStateReady);
    connect(&simulation_loop_timer, &QTimer::timeout, sim_loop_worker, &SimulationLoopWorker::simulate);
    connect(&simulation_model_update_timer, &QTimer::timeout, this,
            &SimulationController::simulationStateReady);


    sim_loop_worker->moveToThread(&simulation_loop_thread);
    simulation_loop_thread.start();

    connect(this, &SimulationController::initSimLoopWorker, sim_loop_worker,
            &SimulationLoopWorker::reInitialize, Qt::DirectConnection);
    emit initSimLoopWorker();
  }

  void SimulationController::clearScenario() {

    //      qDebug() << "Clearing old scenario";
    stopSimulationTimer();
    simulation_loop_thread.quit();
    simulation_loop_thread.wait();
  }

  void SimulationController::startSimulationTimer() {
    if (simulation_loop_timer.isActive()) return;

    simulation_loop_timer.start(16);
    simulation_model_update_timer.start(16);
  }

  void SimulationController::stopSimulationTimer() {
    if (not simulation_loop_timer.isActive()) return;

    simulation_model_update_timer.stop();
    simulation_loop_timer.stop();
  }

  void SimulationController::startStopSimulationTimer() {
    if (simulation_loop_timer.isActive())
      stopSimulationTimer();
    else
      startSimulationTimer();

    emit simulationStateReady();
  }

}   // namespace app
