#include "guiapplication.h"

#include "scenariomodel.h"

// gmlibqtintegration
#include <gmlibqtintegration/geometry/surfaces/tensorproduct/tensorproductsurfacegeometries.h>


// stl
#include <iostream>

int main(int argc, char* argv[])
{

  // Enable Settings
  QCoreApplication::setOrganizationName("UiT The Arctic University of Norway");
  QCoreApplication::setOrganizationDomain("uit.no");
  QCoreApplication::setApplicationName("DTE3607 -- Template Demo");

  // Register custom "gmlib qt integration" qml types
  gmqt::registerCustomGMlib2TPSurfaceGeometryQmlTypes();

  // Construct gui application runtime controller
  app::GuiApplication guiapp(argc, argv);

  // Register scenarios with a solver
  guiapp.registerScenario001AlgStep0();

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

  // Register original test scenarios
  guiapp.registerOriginalStep0TestScenario();
  guiapp.registerOriginalStep1TestScenario();
  guiapp.registerOriginalStep2TestScenario();
  guiapp.registerOriginalStep3aTestScenario();


  // Give runtime control to the qt GuiApplication main tread
  return guiapp.exec();
}
