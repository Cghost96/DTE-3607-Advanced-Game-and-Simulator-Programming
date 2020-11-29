import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QtQuick3D

import SimulatorControl as Sim

Item { id: overlay
  
  Item {
    anchors.top: parent.top
    anchors.right: parent.right
    anchors.bottom: parent.bottom
    anchors.topMargin: 5
    anchors.leftMargin: 5
    anchors.rightMargin: 5
    anchors.bottomMargin: 25
    
    
    width: parent.width / 5.
    height: parent.height / 5.
    
    ColumnLayout {
      anchors.fill: parent
      
      Text{
        Layout.minimumHeight: 20
        font.pixelSize: 24
        text: "Objects"
      }
      
      Rectangle {
        Layout.fillWidth: true
        Layout.fillHeight: true;
        
        color: "transparent"
        
        GridView { id: sim_objects
          anchors.fill: parent
          cellWidth: 20
          cellHeight: 20
          
          clip: true
          
          model: Sim.QmlApi.scenarioModel
          
          delegate:
              Rectangle {
                required property variant object_name
                required property variant velocity
                required property variant frame_origin

                implicitWidth: sim_objects.cellWidth
                implicitHeight: sim_objects.cellHeight
                anchors.margins: 5
                border.width: 2

                color: "pink"
                opacity: 0.5
                radius: 10

                Button { id: sim_objects_delegate_ma
                  anchors.fill: parent
                  opacity: 0

                  ToolTip.visible: down
                  ToolTip.text: "0x"+ object_name + " -- p: " + frame_origin + " -- v: " + velocity
                }
              }
        }
      }
    }
  }
  
}
