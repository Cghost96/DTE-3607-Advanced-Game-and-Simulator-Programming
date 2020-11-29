import QtQuick
import QtQuick3D

import SimulatorControl as Sim


Repeater3D {
  
  model: Sim.QmlApi.scenarioModel
  delegate:
    Model {
      required property variant frame_origin
      required property variant geometry_source
      required property variant geometry_gmlib
      required property variant geometry_type

      position: frame_origin
      geometry: geometry_gmlib
      materials: [
        DefaultMaterial {
          diffuseColor: Qt.hsva(Math.random(),1,1,1);
          opacity: geometry_type === Sim.ScenarioModel.Plane ? 0.8 : 1.0
          cullMode: Material.NoCulling
          lighting: geometry_type === Sim.ScenarioModel.Plane ? DefaultMaterial.NoLighting : DefaultMaterial.FragmentLighting
        }
      ]
    }

//  ScenarioModel.Sphere;
}
