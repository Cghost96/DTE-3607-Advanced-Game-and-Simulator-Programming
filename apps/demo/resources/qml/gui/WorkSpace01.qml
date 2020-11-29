import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QtQuick3D
import QtQuick3D.Helpers

import "../scenes/"

Rectangle {

  readonly property alias stats_view : right.view3d

  Scene01 { id: scene } // Scene -- root node

  CustomView3D { id: right
    anchors.top: parent.top
    anchors.right: parent.right
    width: parent.width * 0.75
    height: parent.height
    camera: scene.cameras.perspective_camera
    importScene: scene

    labelText: "Perspective"

    enableWasdController: true

    blenderIconColumn: 21
    blenderIconRow: 0
    blenderIconAltSource: "qrc:/resources/icons/blender2.8/a22.png"
  }

  ColumnLayout {
    anchors.top: parent.top
    anchors.left: parent.left
    width: parent.width * 0.25
    height: parent.height
    spacing: 0

    CustomView3D {
      id: top_left
      Layout.fillWidth: true
      Layout.fillHeight: true
      camera: scene.cameras.top_orthographic_cam
      importScene: scene

      labelText: "Top"

      blenderIconColumn: 22
      blenderIconRow: 0
      blenderIconAltSource: "qrc:/resources/icons/blender2.8/a23.png"
    }

    CustomView3D {
      id: middle_left
      Layout.fillWidth: true
      Layout.fillHeight: true
      camera: scene.cameras.front_orthographic_cam
      importScene: scene

      labelText: "Front"

      blenderIconColumn: 22
      blenderIconRow: 0
      blenderIconAltSource: "qrc:/resources/icons/blender2.8/a23.png"
    }



    CustomView3D {
      id: bottom_left
      Layout.fillWidth: true
      Layout.fillHeight: true
      camera: scene.cameras.right_orthographic_cam
      importScene: scene

      labelText: "Right"

      blenderIconColumn: 22
      blenderIconRow: 0
      blenderIconAltSource: "qrc:/resources/icons/blender2.8/a23.png"
    }

    Rectangle {
      Layout.maximumHeight: 20
      Layout.fillWidth: true
      border.width: 1
      color: "#003349"
      implicitHeight: slider.height
      implicitWidth: slider.width

      Slider {
        id: slider
        anchors.fill: parent
        orientation: Qt.Horizontal
        value: scene.cameras.ortho_magnification
        stepSize: 0.5
        from: 1.0
        to: 10.0
        onValueChanged: scene.cameras.ortho_magnification = value
      }
    }
  }
  
}
