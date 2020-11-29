import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QtQuick3D
import QtQuick3D.Helpers

import Qt.labs.settings

import SimulatorControl as Sim

import "components/"
import "gui/"

ApplicationWindow {
  id:      window
  width:   1280
  height:  720
  visible: true
  color: "#848895"

  header: ApplicationWindowHeaderToolBar {}

  footer: ApplicationWindowFooterToolBar {}

  Item {
    id: glob

    // overlay
    property font ofont_h : Qt.font( {'bold':true, 'pixelSize':16} )

    // header bar
    property font hfont_h : Qt.font( {'bold':true, 'pixelSize':14} )
    property font hfont_t : Qt.font( {'bold':false, 'pixelSize':12} )

    // footer bar
    property font ffont_h : Qt.font( {'bold':true, 'pixelSize':12} )
    property font ffont_t : Qt.font( {'bold':false, 'pixelSize':10} )

    property View3D stats_view : null

    function fmtNum(number,precision = 2) {
      return number.toFixed(precision)
    }

    function quit() {
      Qt.quit();
    }

    Shortcut {
      sequence: "Ctrl+Q"
      onActivated: glob.quit()
    }
    Shortcut {
      sequence: "Ctrl+R"
      onActivated: Sim.QmlApi.startSimulator()
    }
    Shortcut {
      sequence: "Ctrl+S"
      onActivated: Sim.QmlApi.stopSimulator()
    }

    Shortcut {
      sequence: "Ctrl+P"
      onActivated: {
        var rs = view3d.renderStats
        console.debug("Render stats")
        console.debug("  fps:            " + rs.fps)
        console.debug("  sync time:      " + rs.syncTime)
        console.debug("  render time:    " + rs.renderTime)
        console.debug("  frame time:     " + rs.frameTime)
        console.debug("  max frame time: " + rs.maxFrameTime)
      }
    }
  }

  Item { id: root
    anchors.fill: parent

    states: [
      State { name: "Workspace01"
        PropertyChanges {
          target: ws01
          visible: true
          enabled: true
        }
        PropertyChanges {
          target: glob
          stats_view: ws01.stats_view
        }
      },
      State { name: "Workspace02"
        PropertyChanges {
          target: ws02
          visible: true
          enabled: true
        }
        PropertyChanges {
          target: glob
          stats_view: ws02.stats_view
        }
      }
    ]

    state: "Workspace01"

    WorkSpace01 {
      anchors.fill: parent
      enabled: false
      visible: false
      id: ws01
    }

    WorkSpace02 {
      anchors.fill: parent
      enabled: false
      visible: false
      id: ws02
    }

  }

}




