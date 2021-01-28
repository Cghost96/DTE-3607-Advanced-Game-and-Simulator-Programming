import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Qt.labs.settings

import SimulatorControl as Sim

import "../components/"

ToolBar {
  RowLayout {
    anchors.fill: parent
    anchors.leftMargin: 5
    anchors.rightMargin: 5
    
    ToolButton { id: quit_toolbutton
      //            text: qsTr("^^,"); font: glob.hfont_h
      onClicked: glob.quit()
      Layout.minimumWidth: 4 * height
      Layout.fillHeight: true
      Image {
        anchors.fill: parent
        fillMode: Image.PreserveAspectFit
        id: snap
        //          source: "qrc:/resources/gfx/uit_logo/uit_segl_eng_sort_960px.png"
        //          source: "qrc:/resources/gfx/uit_logo/uit_logo_eng_bla_rgb.png"
        //          source: "qrc:/resources/gfx/uit_logo/uit_logo_eng_2l_bla_rgb.png"
        source: "qrc:/resources/gfx/uit_logo/uit_logo_eng_2l_hvit.png"
      }
    }
    Item{Layout.fillWidth: true}
    ToolButton { // text: qsTr("Work"); font: glob.hfont_h
      BlenderIconImage {
        anchors.fill: parent
        subImgColumn: 19
        subImgRow: 7
        altSource: "qrc:/resources/icons/blender2.8/z1.png"
      }
      onClicked: root.state = "Workspace01"
    }
    ToolSeparator{}
    ToolButton { // text: qsTr("Work (textured spheres)"); font: glob.hfont_h
      BlenderIconImage {
        anchors.fill: parent
        subImgColumn: 3
        subImgRow: 1
        altSource: "qrc:/resources/icons/blender2.8/aa2.png"
      }
      onClicked: root.state = "Workspace02"
    }
    ToolSeparator{}
    ToolButton { // text: qsTr("Demo"); font: glob.hfont_h
      BlenderIconImage {
        anchors.fill: parent
        subImgColumn: 3
        subImgRow: 1
        altSource: "qrc:/resources/icons/blender2.8/aa6.png"
      }
      onClicked: root.state = "Workspace03"
    }
    //      ToolSeparator{}
    //      ToolButton { text: qsTr("Fancy"); font: glob.hfont_h
    //        onClicked: root.state = "Workspace03"
    //      }
    Item{Layout.fillWidth: true}
    ToolButton { // text: qsTr("Start/Stop"); font: glob.hfont_h
      BlenderIconImage {
        anchors.fill: parent
        subImgColumn: 3
        subImgRow: 9
        altSource: "qrc:/resources/icons/blender2.8/j4.png"
      }
      onClicked: Sim.QmlApi.startStopSimulator()
    }
    
    ToolSeparator{}
    ToolButton {// text: qsTr("Reset"); font: glob.hfont_h
      BlenderIconImage {
        anchors.fill: parent
        subImgColumn: 16
        subImgRow: 3
        altSource: "qrc:/resources/icons/blender2.8/d17.png"
      }
      onClicked: Sim.QmlApi.resetSimulator(scenarios_cb.currentText)
    }
    ComboBox{ id: scenarios_cb
      Layout.minimumWidth: 250
      font: glob.hfont_t
      model: Sim.QmlApi.sortedScenarioListModel
      textRole: "scenario_name"
      
      Settings {
        property alias currentIndex : scenarios_cb.currentIndex
      }
    }
  }
}
