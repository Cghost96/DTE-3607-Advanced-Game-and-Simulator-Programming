import QtQuick
import QtQuick3D

import SimulatorControl as Sim


Repeater3D { id: root



  model: Sim.QmlApi.scenarioModel
  delegate:
    Model {
      required property variant frame_origin
      required property variant euler_rotation
      required property variant geometry_source
      required property variant geometry_gmlib
      required property variant geometry_type

      position: frame_origin
      geometry: geometry_gmlib
      eulerRotation: euler_rotation

      DefaultMaterial { id: plane_material
        diffuseMap: Texture{ source:"qrc:/resources/gfx/quick3d_quickball_example/grass.jpg"}
        normalMap: Texture{ source:"qrc:/resources/gfx/quick3d_quickball_example/grass_n.jpg"}
        opacity: 0.4
        cullMode: Material.NoCulling
      }

      DefaultMaterial { id: sphere_material
        diffuseMap: Texture{ source:"qrc:/resources/gfx/quick3d_quickball_example/ball.jpg"}
        normalMap: Texture{ source:"qrc:/resources/gfx/quick3d_quickball_example/ball_n.jpg"}
        opacity: 1.0
        cullMode: Material.NoCulling
        lighting: DefaultMaterial.FragmentLighting
      }

      DefaultMaterial { id: bezier_surf_material
        diffuseMap: Texture{ source:"qrc:/resources/gfx/quick3d_quickball_example/grass.jpg"}
        opacity: 1.0
        cullMode: Material.NoCulling
        lighting: DefaultMaterial.FragmentLighting
      }

      CustomMaterial { id: fun_material_vobl
        shadingMode: CustomMaterial.Shaded
        vertexShader: "qrc:/resources/shaders/quick3d_custommaterial_example/material_distortion.vert"
        fragmentShader: "qrc:/resources/shaders/quick3d_custommaterial_example/material_customlights.frag"
        cullMode: Material.NoCulling
        property real uTime: 0.2
        property real uAmplitude: 0.5
        property color uDiffuse: "yellow"
        property real uShininess: 50
        NumberAnimation on uTime { from: 0.0; to: 31.4; duration: 10000; loops: -1 }
      }

      materials: {
        if(geometry_type === Sim.ScenarioModel.Plane) return plane_material
        else if(geometry_type === Sim.ScenarioModel.Sphere) return sphere_material
        else if(geometry_type === Sim.ScenarioModel.BezierSurface) return bezier_surf_material
        else return fun_material_vobl
      }
    }

//  ScenarioModel.Sphere;
}
