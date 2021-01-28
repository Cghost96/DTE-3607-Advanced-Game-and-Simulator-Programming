import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QtQuick3D
import QtQuick3D.Helpers

Node { id: scene

  property alias cameras : cameras
  
  DefaultLighting {
  }

  DefaultCameras {
    id: cameras
  }

  Loader3D {
    asynchronous: true
    source: "ScenarioModelView02.qml"
  }

  DefaultMaterial { id: grid_material
    lighting: DefaultMaterial.NoLighting
    diffuseColor: "lightGray"
  }

  AxisHelper {
    scale: Qt.vector3d(.05,.05,.05);
    position: Qt.vector3d(-50,0,-50)
    enableXYGrid: false
    enableXZGrid: false
    enableYZGrid: false
  }

  GridGeometry { id: grid_geometry
    horizontalLines: 11
    verticalLines: 11
    horizontalStep: 10
    verticalStep: 10
  }

  GridGeometry { id: grid_geometry_fine
    horizontalLines: 61
    verticalLines: 61
    horizontalStep: 1
    verticalStep: 1
  }


  Model { id: xz_fine
    geometry: grid_geometry_fine
    materials: [grid_material]
    eulerRotation.x: -90
    position: Qt.vector3d(0,0,0)
  }

  Model { id: xz
    geometry: grid_geometry
    materials: [grid_material]
    eulerRotation.x: -90
    position: Qt.vector3d(0,0,0)
  }


  Model { id: xy
    geometry: grid_geometry
    materials: [grid_material]
    position: Qt.vector3d(0,50,-50)
  }


  Model { id: yz
    geometry: grid_geometry
    materials: [grid_material]
    eulerRotation.y: 90
    position: Qt.vector3d(-50,50,0)
  }
}
