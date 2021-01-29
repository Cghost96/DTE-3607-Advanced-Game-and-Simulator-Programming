#include "guiapplication.h"

// simulator
#include "types.h"
#include "simulationcontroller.h"
#include "scenariowrapper.h"

// coldet
#include <coldet/utils/scenario_factories.h>
#include <coldet/solver_development/step0.h>
#include <coldet/solver_development/step1.h>
#include <coldet/solver_development/step2.h>
#include <coldet/solver_development/step3a.h>
#include <coldet/solver_development/step3b.h>
#include <coldet/solver_development/step4.h>
#include <coldet/solver_development/testing_solvers.h>
#include <coldet/mysolvers/rolling.h>

namespace app {

  GuiApplication::GuiApplication(int& argc, char** argv) : QGuiApplication(argc, argv) {
    /**
     Setup and Init GUI
     */

    // MVC -- model
    m_scenariomodel     = new ScenarioModel;
    m_scenariolistmodel = new ScenarioListModel(m_scenarios);

    // MVC -- Sim controller  -- com
    connect(&m_sim_controller, &SimulationController::simulationStateReady, m_scenariomodel,
            &ScenarioModel::updateScene);
    connect(&m_sim_controller, &SimulationController::relayReadySimulationFrameTimings, m_scenariomodel,
            &ScenarioModel::updateSimulatinFrameTimings);

    // MVC -- QML API -- singleton
    m_sim_singleton = new QmlApi(m_scenariomodel, m_scenariolistmodel);

    connect(m_sim_singleton, &QmlApi::startStopSimulator, &m_sim_controller,
            &SimulationController::startStopSimulationTimer);

    connect(m_sim_singleton, &QmlApi::resetSimulator, this, &GuiApplication::resetScenario);

    // Register QML types
    auto const qml_package_name   = "SimulatorControl";
    auto const qml_package_vmajor = 1;
    auto const qml_package_vminor = 0;
    qmlRegisterSingletonInstance<QmlApi>(qml_package_name, qml_package_vmajor, qml_package_vminor, "QmlApi",
                                         m_sim_singleton.data());
    qmlRegisterUncreatableType<ScenarioModel>(qml_package_name, qml_package_vmajor, qml_package_vminor,
                                              "ScenarioModel", "For the enums");

    // Set render surface options (QQuick3D)
    //  QQuickWindow::setGraphicsApi(QSGRendererInterface::VulkanRhi);
    auto const no_samples = 4;
    QSurfaceFormat::setDefaultFormat(QQuick3D::idealSurfaceFormat(no_samples));

    // Create a qml engin and load our scene
    m_engine.load(QUrl(QStringLiteral("qrc:/resources/qml/main.qml")));
    //  if (m_engine.rootObjects().isEmpty()) return -1;
  }

  void GuiApplication::doResetScenario(GuiApplication::ScenarioWPtr scenario) {
    m_sim_controller.resetScenario(scenario);
    m_scenariomodel->resetScenario(scenario);
  }


  void GuiApplication::resetScenario(QString scenario_name) {
    if (not m_scenarios.contains(scenario_name.toStdString())) {
      qDebug() << "NOT loading scenario: " << scenario_name;
      return;
    }

    qDebug() << "Loading scenario: " << scenario_name;
    doResetScenario(m_scenarios.at(scenario_name.toStdString()));
  }

  template <typename ConstructorLambda_T, typename SolverLambda_T>
  void GuiApplication::registerScenario(const std::string& name, ConstructorLambda_T ctr_fn,
                                        SolverLambda_T slv_fn) {
    m_scenarios[name]
      = std::make_shared<ScenarioWrapper<ConstructorLambda_T, SolverLambda_T>>(ctr_fn, slv_fn);
    m_scenariolistmodel->update();
  }

  void GuiApplication::registerScenario001AlgStep0() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario001(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step0::solve(scenario, time_step);
    };

    registerScenario("Scenario 001 - AlgStep 0", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario002AlgStep0() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario002(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step0::solve(scenario, time_step);
    };

    registerScenario("Scenario 002 - AlgStep 0", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario002AlgStep1() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario002(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step1::solve(scenario, time_step);
    };

    registerScenario("Scenario 002 - AlgStep 1", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario003AlgStep1() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario003(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step1::solve(scenario, time_step);
    };

    registerScenario("Scenario 003 - AlgStep 1", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario003AlgStep2() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario003(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step2::solve(scenario, time_step);
    };

    registerScenario("Scenario 003 - AlgStep 2", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario004AlgStep1() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario004(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step1::solve(scenario, time_step);
    };

    registerScenario("Scenario 004 - AlgStep 1", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario004AlgStep2() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario004(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step2::solve(scenario, time_step);
    };

    registerScenario("Scenario 004 - AlgStep 2", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario004AlgStep3a() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario004(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3a::solve(scenario, time_step);
    };

    registerScenario("Scenario 004 - AlgStep 3a", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario005AlgStep3a() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario005(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3a::solve(scenario, time_step);
    };

    registerScenario("Scenario 005 - AlgStep 3a", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario006AlgStep3a() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario006(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3a::solve(scenario, time_step);
    };

    registerScenario("Scenario 006 - AlgStep 3a", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario007AlgStep3a() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario007(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3a::solve(scenario, time_step);
    };

    registerScenario("Scenario 007 - AlgStep 3a", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario008AlgStep3a() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario008(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3a::solve(scenario, time_step);
    };

    registerScenario("Scenario 008 - AlgStep 3a", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario009AlgStep3b() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario009(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3b::solve(scenario, time_step);
    };

    registerScenario("Scenario 009 - AlgStep 3b", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario010AlgStep3b() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario010(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3b::solve(scenario, time_step);
    };

    registerScenario("Scenario 010 - AlgStep 3b", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario011AlgStep3b() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario011(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3b::solve(scenario, time_step);
    };

    registerScenario("Scenario 011 - AlgStep 3b", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario012AlgStep3a() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario012(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3a::solve(scenario, time_step);
    };

    registerScenario("Scenario 012 - AlgStep 3a", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario013AlgStep3b() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario013(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3b::solve(scenario, time_step);
    };

    registerScenario("Scenario 013 - AlgStep 3b", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario014AlgStep3b() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario014(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3b::solve(scenario, time_step);
    };

    registerScenario("Scenario 014 - AlgStep 3b", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario015AlgStep3b() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario015(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3b::solve(scenario, time_step);
    };

    registerScenario("Scenario 015 - AlgStep 3b", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario016AlgStep3b() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario016(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3b::solve(scenario, time_step);
    };

    registerScenario("Scenario 016 - AlgStep 3b", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario017AlgStep3b() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario017(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3b::solve(scenario, time_step);
    };

    registerScenario("Scenario 017 - AlgStep 3b", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario018AlgStep3b() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario018(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3b::solve(scenario, time_step);
    };

    registerScenario("Scenario 018 - AlgStep 3b", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario019AlgStep3b() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario019(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3b::solve(scenario, time_step);
    };

    registerScenario("Scenario 019 - AlgStep 3b", ctr_fn, slv_fn);
  }

  void GuiApplication::registerScenario019AlgStep4() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructScenario019(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step4::solve(scenario, time_step);
    };

    registerScenario("Scenario 019 - AlgStep 4", ctr_fn, slv_fn);
  }

  void GuiApplication::registerOriginalStep0TestScenario() {
    auto ctr_fn
      = []() { return dte3607::coldet::utils::scenario_factories::constructOriginalStep0TestScenario(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step0::solve(scenario, time_step);
    };

    registerScenario("Orig Step0 Test Scenario", ctr_fn, slv_fn);
  }

  void GuiApplication::registerOriginalStep1TestScenario() {
    auto ctr_fn
      = []() { return dte3607::coldet::utils::scenario_factories::constructOriginalStep1TestScenario(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step1::solve(scenario, time_step);
    };

    registerScenario("Orig Step1 Test Scenario", ctr_fn, slv_fn);
  }

  void GuiApplication::registerOriginalStep2TestScenario() {
    auto ctr_fn
      = []() { return dte3607::coldet::utils::scenario_factories::constructOriginalStep2TestScenario(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step2::solve(scenario, time_step);
    };

    registerScenario("Orig Step2 Test Scenario", ctr_fn, slv_fn);
  }

  void GuiApplication::registerOriginalStep3aTestScenario() {
    auto ctr_fn
      = []() { return dte3607::coldet::utils::scenario_factories::constructOriginalStep3aTestScenario(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::step3a::solve(scenario, time_step);
    };

    registerScenario("Orig Step3a Test Scenario", ctr_fn, slv_fn);
  }

  void GuiApplication::registerComponentTestingScenario01() {
    auto ctr_fn
      = []() { return dte3607::coldet::utils::scenario_factories::constructComponentTestingScenario01(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::testing::componentTestingSolverRotationOfSpecificObjects01(scenario,
                                                                                              time_step);
    };

    registerScenario("Component Test Scenario 01", ctr_fn, slv_fn);
  }

  void GuiApplication::registerComponentTestingScenario02() {
    auto ctr_fn
      = []() { return dte3607::coldet::utils::scenario_factories::constructComponentTestingScenario02(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::testing::componentTestingSolverPassive(scenario, time_step);
    };

    registerScenario("Component Test Scenario 02", ctr_fn, slv_fn);
  }

  void GuiApplication::registerComponentTestingScenario03() {
    auto ctr_fn
      = []() { return dte3607::coldet::utils::scenario_factories::constructComponentTestingScenario03(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::solver_dev::testing::componentTestingSolverPassive(scenario, time_step);
    };

    registerScenario("Component Test Scenario 03", ctr_fn, slv_fn);
  }

  /* --------------------------------------------------------------------------------- */
  /* ------------------------------- PROJECT SCENARIOS ------------------------------- */
  /* --------------------------------------------------------------------------------- */

  // ------------ Registration template -------------
  //  void GuiApplication::registerScenario/*scenario name*/()
  //  {
  //    auto ctr_fn = []() {
  //      return dte3607::coldet::utils::scenario_factories::/*name of factory function*/();
  //    };
  //    auto slv_fn = [](auto& scenario, auto const& time_step) {
  //      dte3607::coldet::mysolvers::/*step name*/::solve(scenario, time_step);
  //    };

  //    registerScenario("[Scenario name]", ctr_fn, slv_fn);
  //  }

  void GuiApplication::registerScenarioGaltonInitTest() {
    auto ctr_fn = []() { return dte3607::coldet::utils::scenario_factories::constructGaltonInitTest(); };
    auto slv_fn = [](auto& scenario, auto const& time_step) {
      dte3607::coldet::mysolvers::rolling::solve(scenario, time_step);
    };

    registerScenario("Galton - init test", ctr_fn, slv_fn);
  }

}   // namespace app
