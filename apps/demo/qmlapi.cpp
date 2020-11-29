#include "qmlapi.h"

namespace app
{

  QmlApi::QmlApi(QPointer<ScenarioModel>     scenariomodel,
                 QPointer<ScenarioListModel> scenariolistmodel, QObject* parent)
    : QObject(parent), m_scenariomodel(scenariomodel),
      m_scenariolistmodel(scenariolistmodel),
      m_sorted_scenariolistmodel(new QSortFilterProxyModel)
  {
    m_sorted_scenariolistmodel->setSourceModel(m_scenariolistmodel);
  }

  ScenarioModel* QmlApi::scenarioModel() const { return m_scenariomodel; }

  ScenarioListModel* QmlApi::scenarioListModel() const
  {
    return m_scenariolistmodel;
  }

  QSortFilterProxyModel* QmlApi::sortedScenarioListModel() const
  {
    return m_sorted_scenariolistmodel;
  }

}   // namespace app
