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
      required property variant fixed_sphere

      position: frame_origin
      geometry: geometry_gmlib
      eulerRotation: euler_rotation

      DefaultMaterial { id: plane_material
        diffuseColor: "white"
        opacity: 0.1
        cullMode: Material.NoCulling
        lighting: DefaultMaterial.NoLighting
      }

      DefaultMaterial { id: fixed_plane_material
        diffuseColor: "gray"
        opacity: 0.2
        cullMode: Material.NoCulling
        lighting: DefaultMaterial.NoLighting
      }

      DefaultMaterial { id: sphere_material
        diffuseColor: "darkorange"
        opacity: 1.0
        cullMode: Material.NoCulling
        lighting: DefaultMaterial.FragmentLighting
      }

      DefaultMaterial { id: fixed_sphere_material
        diffuseColor: "azure"
        opacity: 1.0
        cullMode: Material.NoCulling
        lighting: DefaultMaterial.FragmentLighting
      }

      DefaultMaterial { id: bezier_surf_material
        diffuseColor: Qt.hsva(Math.random(),1,1,1);
        opacity: 0.8
        cullMode: Material.NoCulling
      }

      CustomMaterial { id: fun_material_transp
        shadingMode: CustomMaterial.Shaded
        fragmentShader: "qrc:/resources/shaders/quick3d_custommaterial_example/material_transparent.frag"
        cullMode: Material.NoCulling
      }

      CustomMaterial { id: fun_material_spec
        shadingMode: CustomMaterial.Shaded
        fragmentShader: "qrc:/resources/shaders/quick3d_custommaterial_example/material_customspecular.frag"
        cullMode: Material.NoCulling
        property color uDiffuse: "green"
        property real uShininess: 150
      }

      materials: {
        if(geometry_type === Sim.ScenarioModel.Plane) return plane_material
        else if(geometry_type === Sim.ScenarioModel.Sphere && fixed_sphere === true) return fixed_sphere_material
        else if(geometry_type === Sim.ScenarioModel.Sphere) return sphere_material
        else if(geometry_type === Sim.ScenarioModel.BezierSurface) return bezier_surf_material
        else if(geometry_type === Sim.ScenarioModel.LimitedPlane) return fixed_plane_material
        else return fun_material_spec
      }
    }

//  ScenarioModel.Sphere;
}
