#include "guiapplication.h"

#include "scenariomodel.h"

// gmlibqtintegration
#include <gmlibqtintegration/geometry/surfaces/tensorproduct/tensorproductsurfacegeometries.h>


// stl
#include <iostream>

int main(int argc, char* argv[]) {

  // Enable Settings
  QCoreApplication::setOrganizationName("UiT The Arctic University of Norway");
  QCoreApplication::setOrganizationDomain("uit.no");
  QCoreApplication::setApplicationName("DTE3607 -- Template Demo");

  // Register custom "gmlib qt integration" qml types
  gmqt::registerCustomGMlib2TPSurfaceGeometryQmlTypes();

  // Construct gui application runtime controller
  app::GuiApplication guiapp(argc, argv);

  // Register scenarios with a solver
  /*  guiapp.registerScenario001AlgStep0();

    guiapp.registerScenario002AlgStep0();
    guiapp.registerScenario002AlgStep1();

    guiapp.registerScenario003AlgStep1();
    guiapp.registerScenario003AlgStep2();

    guiapp.registerScenario004AlgStep1();
    guiapp.registerScenario004AlgStep2();
    guiapp.registerScenario004AlgStep3a();

    guiapp.registerScenario005AlgStep3a();

    guiapp.registerScenario006AlgStep3a();

    guiapp.registerScenario007AlgStep3a();

    guiapp.registerScenario008AlgStep3a();

    guiapp.registerScenario009AlgStep3b();

    guiapp.registerScenario010AlgStep3b();

    guiapp.registerScenario011AlgStep3b();

    guiapp.registerScenario012AlgStep3a();

    guiapp.registerScenario013AlgStep3b();
    guiapp.registerScenario014AlgStep3b();
    guiapp.registerScenario015AlgStep3b();
    guiapp.registerScenario016AlgStep3b();
    guiapp.registerScenario017AlgStep3b();
    guiapp.registerScenario018AlgStep3b();
    guiapp.registerScenario019AlgStep3b();

    // Register original test scenarios
    guiapp.registerOriginalStep0TestScenario();
    guiapp.registerOriginalStep1TestScenario();
    guiapp.registerOriginalStep2TestScenario();
    guiapp.registerOriginalStep3aTestScenario();

   Register component testing scenarios
    guiapp.registerComponentTestingScenario01();
    guiapp.registerComponentTestingScenario02();
    guiapp.registerComponentTestingScenario03();*/

  /* --------------------------------------------------------------------------------- */
  /* ------------------------------- PROJECT SCENARIOS ------------------------------- */
  /* --------------------------------------------------------------------------------- */

  /* --------------------GALTON-------------------- */
  /* --------------------NO TOUCHING ALLOWED IN FUNNEL-------------------- */
  guiapp.registerScenarioGaltonTest01();
  guiapp.registerScenarioGaltonTest02();
  guiapp.registerScenarioGaltonTest03();
  guiapp.registerScenarioGaltonTest04();
  guiapp.registerScenarioGaltonTest05();

  /* --------------------TOUCHING ALLOWED IN FUNNEL-------------------- */
  guiapp.registerScenarioGaltonTest06();
  guiapp.registerScenarioGaltonTest07();

  /* ----------------TOUCHING ALLOWED IN FUNNEL AND TIMER---------------- */
  guiapp.registerScenarioGaltonTest08();
  guiapp.registerScenarioGaltonTest09();
  guiapp.registerScenarioGaltonTest10();

  // Give runtime control to the qt GuiApplication main tread
  return guiapp.exec();
}
