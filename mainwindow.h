#pragma once
#include "mavlinkmanager.h"
#include <QMainWindow>

class QLabel;
class QTableWidget;
class QListWidget;
class QPushButton;
class QDoubleSpinBox;
class QGraphicsEllipseItem;
class QGraphicsPathItem;
class QLineSeries;
class QValueAxis;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow();
private:
    void refresh();
    int selectedKey() const;
    void addEvent(const QString &type, const QString &message);

    TelemetryModel model_;
    MavlinkManager manager_{model_};
    QLabel *link_, *quality_, *sender_, *position_, *command_;
    QTableWidget *table_;
    QListWidget *journal_;
    QPushButton *open_, *close_, *takeoff_, *land_;
    QDoubleSpinBox *altitude_;
    QLineSeries *series_;
    QValueAxis *timeAxis_, *heightAxis_;
    QGraphicsEllipseItem *marker_;
    QGraphicsPathItem *trail_;
};
