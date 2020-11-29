import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Qt.labs.settings

import SimulatorControl as Sim

ToolBar {
  RowLayout {
    anchors.fill: parent
    anchors.leftMargin: 5
    anchors.rightMargin: 5
    
    Label { font: glob.ffont_h; text: "Frame stats [ms]: "}
    Label { font: glob.ffont_t; text: "FPS " + glob.fmtNum(glob.stats_view.renderStats.fps,0)}
    ToolSeparator{}
    Label { font: glob.ffont_t; text: "S " + glob.fmtNum(glob.stats_view.renderStats.syncTime)}
    ToolSeparator{}
    Label { font: glob.ffont_t; text: "R " + glob.fmtNum(glob.stats_view.renderStats.renderTime) + " (P: " + glob.fmtNum(glob.stats_view.renderStats.renderPrepareTime) + ")"}
    ToolSeparator{}
    Label { font: glob.ffont_t; text: "F " + glob.fmtNum(glob.stats_view.renderStats.frameTime) + " (M: " + glob.fmtNum(glob.stats_view.renderStats.maxFrameTime) + ")"}
    Item{Layout.fillWidth: true}
    Label { font: glob.ffont_h; text: "No. obj: " + glob.fmtNum(Sim.QmlApi.scenarioModel.noObjects,0)}
    Item{width: 50}
    Label { font: glob.ffont_h; text: "Sim stats [ms]: "}
    Label { font: glob.ffont_t; text: "Timestep " + glob.fmtNum(Sim.QmlApi.scenarioModel.simLoopTimestepAverage)}
    ToolSeparator{}
    Label { font: glob.ffont_t; text: "Simtime " + glob.fmtNum(Sim.QmlApi.scenarioModel.simLoopTimestepAverage)}
  }
}
