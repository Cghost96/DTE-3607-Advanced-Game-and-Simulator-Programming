#ifndef DTE3607_QMLAPI_H
#define DTE3607_QMLAPI_H


#include "scenariomodel.h"
#include "scenariolistmodel.h"

// qt
#include <QObject>
#include <QSortFilterProxyModel>


namespace app
{

  class QmlApi : public QObject   // Singleton
  {
    Q_OBJECT
    Q_PROPERTY(ScenarioModel* scenarioModel READ scenarioModel CONSTANT)
    Q_PROPERTY(
      ScenarioListModel* scenarioListModel READ scenarioListModel CONSTANT)
    Q_PROPERTY(QSortFilterProxyModel* sortedScenarioListModel READ
                 sortedScenarioListModel CONSTANT)


  public:
    explicit QmlApi(QPointer<ScenarioModel>     scenariomodel,
                    QPointer<ScenarioListModel> scenariolistmodel,
                    QObject*                    parent = nullptr);

    ScenarioModel*         scenarioModel() const;
    ScenarioListModel*     scenarioListModel() const;
    QSortFilterProxyModel* sortedScenarioListModel() const;

  private:
    QPointer<ScenarioModel>         m_scenariomodel;
    QPointer<ScenarioListModel>     m_scenariolistmodel;
    QPointer<QSortFilterProxyModel> m_sorted_scenariolistmodel;

  signals:
    void startStopSimulator();
    void resetSimulator(QString scenario_name);
  };

}   // namespace app


#endif   // DTE3607_QMLAPI_H
