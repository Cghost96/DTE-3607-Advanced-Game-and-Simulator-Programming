#ifndef GUIAPPLICATION_H
#define GUIAPPLICATION_H


#include "simulationcontroller.h"
#include "scenariowrapper.h"
#include "scenariomodel.h"
#include "qmlapi.h"



// qt
#include <QGuiApplication>

// qt
//#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickWindow>

#include <QtGui>
#include <QtQuick3D/QQuick3D>
#include <QPointer>

namespace app
{


  class GuiApplication : public QGuiApplication {
    Q_OBJECT

  public:
    GuiApplication(int& argc, char** argv);
    ~GuiApplication() override = default;


    void registerScenario001AlgStep0();

    void registerScenario002AlgStep0();
    void registerScenario002AlgStep1();

    void registerScenario003AlgStep1();
    void registerScenario003AlgStep2();

    void registerScenario004AlgStep1();
    void registerScenario004AlgStep2();
    void registerScenario004AlgStep3a();

    void registerScenario005AlgStep3a();

    void registerScenario006AlgStep3a();

    void registerScenario007AlgStep3a();

    void registerScenario008AlgStep3a();

    void registerScenario009AlgStep3b();
    void registerScenario010AlgStep3b();
    void registerScenario011AlgStep3b();

    void registerScenario012AlgStep3a();


    // Original test scenarios and alg steps [1 - 3a]
    void registerOriginalStep0TestScenario();
    void registerOriginalStep1TestScenario();
    void registerOriginalStep2TestScenario();
    void registerOriginalStep3aTestScenario();

  private:
    QQmlApplicationEngine m_engine;
    SimulationController  m_sim_controller;

    QPointer<ScenarioModel>     m_scenariomodel;
    QPointer<ScenarioListModel> m_scenariolistmodel;
    QPointer<QmlApi>            m_sim_singleton;



    using ScenarioPtr  = std::shared_ptr<ScenarioWrapperBase>;
    using ScenarioWPtr = std::weak_ptr<ScenarioWrapperBase>;
    std::map<std::string, ScenarioPtr> m_scenarios;


    template <typename ConstructorLambda_T, typename SolverLambda_T>
    void registerScenario(std::string const& name, ConstructorLambda_T ctr_fn,
                          SolverLambda_T slv_fn);

    void doResetScenario(ScenarioWPtr scenario);

  public slots:
    void resetScenario(QString scenario_name);
  };

}   // namespace app



#endif   // GUIAPPLICATION_H
