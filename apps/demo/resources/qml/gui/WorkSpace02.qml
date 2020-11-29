import QtQuick
import QtQuick.Controls
import QtQuick3D

import "./"
import "../scenes/"

Rectangle{ id: scene02

  readonly property alias stats_view : view3d.view3d


  Scene02 { id: scene } // Scene -- root node

  CustomView3D { id: view3d
    anchors.fill: parent
    camera: scene.cameras.perspective_camera
    importScene: scene

    labelText: "Perspective"

    enableWasdController: true

    blenderIconColumn: 21
    blenderIconRow: 0
    blenderIconAltSource: "qrc:/resources/icons/blender2.8/a22.png"

//    Overlay {
//      id: overlay
//      anchors.fill: parent
//      z: 1
//    }
  }



//  Rectangle {
//    anchors.fill: scene02
//    z: view3d.z+1
//    visible: !view3d.focus
////    visible: true

////    color: "transparent"
//    color: "red"
//    border.color: "gray"
//    border.width: 2

//    MouseArea {
//      anchors.fill: parent
//      onClicked: {
//        view3d.forceActiveFocus(Qt.WheelFocus)
//      }
//    }
//  }

}
