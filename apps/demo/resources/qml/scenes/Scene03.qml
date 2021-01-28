import QtQuick
import QtQuick3D

Node { id: scene

  property alias cameras : cameras
  
  DefaultLighting {
  }

  DefaultCameras {
    id: cameras
  }
  Loader3D {
    asynchronous: true
    source: "ScenarioModelView01.qml"
  }
}
