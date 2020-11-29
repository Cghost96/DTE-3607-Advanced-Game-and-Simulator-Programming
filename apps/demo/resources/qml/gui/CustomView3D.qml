import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

import QtQuick3D
import QtQuick3D.Helpers

import "."
import "../components/"

Rectangle { id: root
  color: "#848895"
  border.color: "#003349"
  border.width: 1
  clip: true

  default property alias content: view3d.children

  property alias camera: view3d.camera
  property alias importScene: view3d.importScene
  readonly property alias view3d: view3d

  property alias labelText: label.text

  property alias enableWasdController : wasdController.enabled

  property alias blenderIconColumn: blender_icon.subImgColumn
  property alias blenderIconRow: blender_icon.subImgRow
  property alias blenderIconAltSource: blender_icon.altSource

  View3D { id: view3d
    anchors.fill: parent
    anchors.margins: root.border.width
    importScene: scene
    environment: SceneEnvironment{
      antialiasingQuality: SceneEnvironment.VeryHigh
      antialiasingMode: SceneEnvironment.MSAA
    }

    WasdController { id: wasdController; enabled: false; controlledObject: view3d.camera }
  }
  
  Item { id: label_item
    visible: label.text.length
    anchors.top: parent.top
    anchors.left: parent.left
    anchors.margins: 10
    height: label_row.height + 5
    width: label_row.width + 5


    Rectangle { id: label_rect
      anchors.fill: parent
      border.width: 0
      opacity: 0.05
      radius: 5
      color: "black"

    }

    Row { id: label_row
      spacing: 5
      z: label_rect.z + 1
      anchors.centerIn: label_rect

      BlenderIconImage{ id: blender_icon
        height: label.height
        width: height
      }
      Label { id: label
        color: "#222840"
        font: glob.ofont_h
      }
    }
  }


}
