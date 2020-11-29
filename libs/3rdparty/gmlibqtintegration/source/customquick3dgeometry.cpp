#include <gmlibqtintegration/private/customquick3dgeometry.h>

namespace gmqt
{
  CustomQuick3DGeometry::CustomQuick3DGeometry(QQuick3DObject* parent)
    : QQuick3DGeometry(parent)
  {
    setObjectName(QString("gmlib2_customquick3dgeometry_obj_hash_%1")
                    .arg(std::hash<CustomQuick3DGeometry*>{}(this)));
  }

  void CustomQuick3DGeometry::updateAndMarkDirty()
  {
    update();
    m_dirty = true;
  }

  QSSGRenderGraphObject*
  CustomQuick3DGeometry::updateSpatialNode(QSSGRenderGraphObject* node)
  {
    if (m_dirty) rebuildGeometry();

    node = QQuick3DGeometry::updateSpatialNode(node);
    return node;
  }

}   // namespace gmqt
