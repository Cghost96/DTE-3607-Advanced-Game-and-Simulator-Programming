import QtQuick3D

Node {
  DirectionalLight {
    eulerRotation.x: -45
    eulerRotation.y: 45
  }

  DirectionalLight {
    eulerRotation.x: -225
    eulerRotation.y: 45
  }

  PointLight {
    x: -50
    y: 50
    z: -50
  }
}
