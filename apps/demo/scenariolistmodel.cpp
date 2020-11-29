#include "scenariolistmodel.h"

namespace app
{

  ScenarioListModel::ScenarioListModel(
    const ScenarioListModel::Scenarios& scenarios)
    : m_scenarios{scenarios}
  {
  }

  int ScenarioListModel::rowCount(const QModelIndex&) const
  {
    return int(m_scenarios.size());
  }

  QVariant ScenarioListModel::data(const QModelIndex& index, int role_in) const
  {

    if (not index.isValid()) return {};


    auto const role = Roles(role_in);
    if (role not_eq Roles::ObjectName) return {};

    auto scenarios_itr = std::begin(m_scenarios);
    std::advance(scenarios_itr, index.row());

    return {scenarios_itr->first.c_str()};
  }

  QHash<int, QByteArray> ScenarioListModel::roleNames() const
  {
    return {{int(Roles::ObjectName), "scenario_name"}};
  }

  void ScenarioListModel::update()
  {
    beginResetModel();
    endResetModel();
  }

}   // namespace app
