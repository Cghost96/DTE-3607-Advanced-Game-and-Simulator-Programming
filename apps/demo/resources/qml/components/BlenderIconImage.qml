import QtQuick

Image{ id: img
  fillMode: Image.PreserveAspectFit

  property string altSource : ""
  readonly property bool useAltSource : altSource.length > 0

// Using SVG -- Fails "sometimes" on windows
  property int subImgColumn: 0
  property int subImgRow: 0

  // Orig Blender Icon SVG -- res x2
  readonly property string svgSource : "qrc:/resources/icons/blender2.8/blender2.8_icons_2019-06-04.svg"
  readonly property int offsetC: 6
  readonly property int offsetR: 16
  readonly property int spacingC: 2
  readonly property int spacingR: 2
  readonly property int subImgW: 40
  readonly property int subImgH: 40
  readonly property int iconSetLoadWidth: 1204
  readonly property int iconSetLoadHeight: 1280

  sourceSize: useAltSource ? undefined : Qt.size(iconSetLoadWidth,iconSetLoadHeight)
  sourceClipRect: useAltSource ? undefined :  Qt.rect(
                    offsetC + subImgColumn * (subImgW + spacingC),
                    offsetR + subImgRow * (subImgH + spacingR),
                    subImgW,
                    subImgH
                    )
  source:  useAltSource ? altSource : svgSource
}
