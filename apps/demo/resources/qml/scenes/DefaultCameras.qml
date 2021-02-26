import QtQuick
import QtQuick3D


Node {
  property alias perspective_camera: p_camera
  property alias top_orthographic_cam: o_camera_top
  property alias front_orthographic_cam: o_camera_front
  property alias left_orthographic_cam: o_camera_left
  property alias right_orthographic_cam: o_camera_right

  property real ortho_magnification : 0.5

  PerspectiveCamera { id: p_camera

    clipFar: 10000.0
//    position: Qt.vector3d(-40, 40, 40)
    position: Qt.vector3d(100, 300, 100)
//    position: Qt.vector3d(0, 0, 0)
    eulerRotation.x: -30

    Component.onCompleted: lookAt( Qt.vector3d(0, 0, 0) )
  }
  
  OrthographicCamera { id: o_camera_top
    position: Qt.vector3d(0, 1000, 0)
    eulerRotation.x: -90
    horizontalMagnification: ortho_magnification
    verticalMagnification: ortho_magnification
  }

  OrthographicCamera { id: o_camera_front
    position: Qt.vector3d(0, 10, 1000)
    horizontalMagnification: ortho_magnification
    verticalMagnification: ortho_magnification
  }

  OrthographicCamera { id: o_camera_left
    position: Qt.vector3d(-1000, 10, 0)
    eulerRotation.y: -90
    horizontalMagnification: ortho_magnification
    verticalMagnification: ortho_magnification
  }

  OrthographicCamera { id: o_camera_right
    position: Qt.vector3d(1000, 10, 0)
    eulerRotation.y: 90
    horizontalMagnification: ortho_magnification
    verticalMagnification: ortho_magnification
  }
}
