#ifndef DTE3607_SCENARIOLISTMODEL_H
#define DTE3607_SCENARIOLISTMODEL_H

#include "scenariowrapper.h"

// qt
#include <QAbstractListModel>


namespace app
{

  class ScenarioListModel : public QAbstractListModel {
    Q_OBJECT

  public:
    using Scenarios
      = std::map<std::string, std::shared_ptr<ScenarioWrapperBase>>;

    ScenarioListModel(Scenarios const& scenarios);

    enum class Roles {
      ObjectName = Qt::UserRole,
    };

    int      rowCount(const QModelIndex& /*index*/) const override;
    QVariant data(const QModelIndex& index,
                  int                role_in = Qt::DisplayRole) const override;
    QHash<int, QByteArray> roleNames() const override;

  private:
    Scenarios const& m_scenarios;

  public slots:
    void update();
  };

}   // namespace app


#endif   // DTE3607_SCENARIOLISTMODEL_H
