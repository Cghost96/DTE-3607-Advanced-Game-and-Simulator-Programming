#include "simulationloopworker.h"

namespace app {

  SimulationLoopWorker::SimulationLoopWorker(std::weak_ptr<ScenarioWrapperBase> scenario,
                                             NanoSeconds                        timestep)
    : m_scenario{scenario}, m_timestep{timestep} {
    m_scenario    = scenario;
    m_initialized = false;
  }

  void SimulationLoopWorker::simulate() {
    std::scoped_lock lock(m_mutex);

    if (not m_initialized) return;

    if (not m_prev_timepoint) {
      m_prev_timepoint = Clock::now();
      return;
    }

    auto scenario = m_scenario.lock();

    if (not scenario) {
      return;
    }


    NanoSeconds time_step          = m_timestep;
    auto const  benchmark_tp_start = std::chrono::system_clock::now();
    {
      using namespace std::chrono_literals;
      if (std::cmp_equal(time_step.count(), 0)) {
        auto const now            = Clock::now();
        auto const prev_timepoint = m_prev_timepoint.value();
        m_prev_timepoint          = now;
        auto variable_time_step   = std::chrono::duration_cast<NanoSeconds>(now - prev_timepoint);
        time_step                 = variable_time_step;
      }



      scenario->solve(time_step);
    }
    auto const benchmark_tp_end = std::chrono::system_clock::now();

    // Benchmark
    auto const benchmark_diff(benchmark_tp_end - benchmark_tp_start);

    // Emit finished signal
    emit simulationFrameTimingsReady(int(std::chrono::duration_cast<MicroSeconds>(time_step).count()),
                                     int(std::chrono::duration_cast<MicroSeconds>(benchmark_diff).count()));
  }

  void SimulationLoopWorker::reInitialize() {
    std::scoped_lock lock(m_mutex);
    auto             scenario = m_scenario.lock();
    scenario->construct();
    m_initialized = true;
    emit initialized();
  }

}   // namespace app
