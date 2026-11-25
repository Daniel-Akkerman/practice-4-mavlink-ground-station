#include "mainwindow.h"
#include "mavlinkmessageparser.h"
#include "ui_mainwindow.h"
#include <QChart>
#include <QChartView>
#include <QDoubleSpinBox>
#include <QGraphicsEllipseItem>
#include <QGraphicsView>
#include <QHeaderView>
#include <QLabel>
#include <QLineSeries>
#include <QListWidget>
#include <QPainterPath>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTableWidget>
#include <QValueAxis>
#include <QVBoxLayout>
#include <algorithm>

namespace {
void tone(QLabel *label, int state)
{
    const auto color = state > 0 ? "#147052" : state < 0 ? "#a72e38" : "#81530b";
    label->setStyleSheet(QString("font-weight: 650; color: %1").arg(color));
}

QPointF project(const QPointF &coordinate)
{
    return {(coordinate.x() - 37.615560) * 170000, -(coordinate.y() - 55.752220) * 225000};
}
}

MainWindow::MainWindow()
{
    Ui::StationWindow ui;
    ui.setupUi(this);
    auto *layout = ui.stationLayout;

    auto *title = new QLabel("Наземная станция · UDP / MAVLink 2");
    title->setStyleSheet("font-size: 23px; font-weight: 650;");
    layout->addWidget(title);
    auto *connection = new QHBoxLayout;
    open_ = new QPushButton("Открыть канал · 14540");
    close_ = new QPushButton("Закрыть канал");
    open_->setObjectName("btnStartLink");
    close_->setObjectName("btnStopLink");
    link_ = new QLabel;
    link_->setObjectName("labelLinkStatus");
    connection->addWidget(open_);
    connection->addWidget(close_);
    connection->addWidget(link_, 1);
    layout->addLayout(connection);
    quality_ = new QLabel("Принято 0  ·  Ошибок 0  ·  Потеряно 0  ·  0 пак./с  ·  Последний пакет —");
    quality_->setObjectName("labelPackets");
    layout->addWidget(quality_);

    table_ = new QTableWidget(0, 7);
    table_->setObjectName("tableFleet");
    table_->setHorizontalHeaderLabels({"№", "System ID", "Component ID", "Адрес", "Порт", "Последний пакет", "Статус"});
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->verticalHeader()->hide();
    table_->setMaximumHeight(122);
    layout->addWidget(table_);

    sender_ = new QLabel;
    position_ = new QLabel;
    layout->addWidget(sender_);
    layout->addWidget(position_);
    auto *visuals = new QHBoxLayout;
    auto *chart = new QChart;
    chart->setTitle("Высота выбранного борта");
    chart->legend()->hide();
    series_ = new QLineSeries;
    series_->setColor(QColor("#2166ad"));
    chart->addSeries(series_);
    timeAxis_ = new QValueAxis;
    heightAxis_ = new QValueAxis;
    timeAxis_->setTitleText("Время, с");
    heightAxis_->setTitleText("Высота, м");
    chart->addAxis(timeAxis_, Qt::AlignBottom);
    chart->addAxis(heightAxis_, Qt::AlignLeft);
    series_->attachAxis(timeAxis_);
    series_->attachAxis(heightAxis_);
    auto *chartView = new QChartView(chart);
    chartView->setObjectName("chartView");
    chartView->setRenderHint(QPainter::Antialiasing);
    visuals->addWidget(chartView, 3);

    auto *mapLayout = new QVBoxLayout;
    mapLayout->addWidget(new QLabel("Карта · локальная координатная сетка"));
    auto *scene = new QGraphicsScene(this);
    scene->setSceneRect(-120, -115, 240, 230);
    scene->setBackgroundBrush(QColor("#edf5fa"));
    for (int coordinate = -100; coordinate <= 100; coordinate += 25) {
        scene->addLine(coordinate, -110, coordinate, 110, QPen(QColor("#c9dce9")));
        scene->addLine(-115, coordinate, 115, coordinate, QPen(QColor("#c9dce9")));
    }
    trail_ = scene->addPath({}, QPen(QColor("#188272"), 2));
    marker_ = scene->addEllipse(-5, -5, 10, 10, QPen(Qt::white), QBrush(QColor("#2166ad")));
    auto *map = new QGraphicsView(scene);
    map->setObjectName("graphicsView");
    map->setRenderHint(QPainter::Antialiasing);
    mapLayout->addWidget(map);
    visuals->addLayout(mapLayout, 2);
    layout->addLayout(visuals, 1);

    auto *commands = new QHBoxLayout;
    altitude_ = new QDoubleSpinBox;
    altitude_->setObjectName("spinTakeoffAltitude");
    altitude_->setRange(10, 500);
    altitude_->setSingleStep(10);
    altitude_->setDecimals(0);
    altitude_->setSuffix(" м");
    altitude_->setValue(50);
    takeoff_ = new QPushButton("Взлёт");
    takeoff_->setObjectName("btnTakeoff");
    land_ = new QPushButton("Посадка");
    land_->setObjectName("btnLand");
    command_ = new QLabel("Команды ещё не отправлялись");
    command_->setObjectName("labelCommandStatus");
    commands->addWidget(new QLabel("Высота взлёта"));
    commands->addWidget(altitude_);
    commands->addWidget(takeoff_);
    commands->addWidget(land_);
    commands->addWidget(command_, 1);
    layout->addLayout(commands);
    layout->addWidget(new QLabel("Журнал связи и команд"));
    journal_ = new QListWidget;
    journal_->setObjectName("listCommandLog");
    journal_->setMaximumHeight(110);
    layout->addWidget(journal_);

    connect(open_, &QPushButton::clicked, this, [this] { manager_.start(); });
    connect(close_, &QPushButton::clicked, &manager_, &MavlinkManager::stop);
    connect(takeoff_, &QPushButton::clicked, this, [this] { manager_.sendCommand(selectedKey(), Mavlink::Takeoff, altitude_->value()); });
    connect(land_, &QPushButton::clicked, this, [this] { manager_.sendCommand(selectedKey(), Mavlink::Land); });
    connect(table_, &QTableWidget::currentCellChanged, this, &MainWindow::refresh);
    connect(&model_, &TelemetryModel::changed, this, &MainWindow::refresh);
    connect(&manager_, &MavlinkManager::channelChanged, this, &MainWindow::refresh);
    connect(&manager_, &MavlinkManager::eventLogged, this, &MainWindow::addEvent);
    connect(&manager_, &MavlinkManager::commandStatus, this, [this](const QString &text, int state) {
        command_->setText(text);
        tone(command_, state);
        refresh();
    });
    connect(&manager_, &MavlinkManager::qualityChanged, this,
        [this](quint64 received, quint64 errors, quint64 lost, int rate, qint64 age) {
            quality_->setText(QString("Принято %1  ·  Ошибок %2  ·  Потеряно %3  ·  %4 пак./с  ·  Последний пакет %5")
                .arg(received).arg(errors).arg(lost).arg(rate)
                .arg(age < 0 ? QString("—") : QString("%1 мс назад").arg(age)));
        });
    resize(1060, 710);
    refresh();
}

int MainWindow::selectedKey() const
{
    const auto *cell = table_->item(table_->currentRow(), 0);
    return cell ? cell->data(Qt::UserRole).toInt() : -1;
}

void MainWindow::refresh()
{
    const auto &vehicles = model_.vehicles();
    const int selected = selectedKey();
    const QSignalBlocker lock(table_);
    table_->setRowCount(vehicles.size());
    int row = 0, selectedRow = 0;
    for (auto it = vehicles.cbegin(); it != vehicles.cend(); ++it, ++row) {
        if (it.key() == selected)
            selectedRow = row;
        const QStringList values = {QString::number(row + 1), QString::number(it->system),
            QString::number(it->component), it->address.toString(),
            it->port ? QString::number(it->port) : QString("—"), it->received.toString("HH:mm:ss"),
            it->online ? "Online" : "Offline"};
        for (int column = 0; column < values.size(); ++column) {
            auto *item = table_->item(row, column);
            if (!item) {
                item = new QTableWidgetItem;
                table_->setItem(row, column, item);
            }
            item->setText(values[column]);
        }
        table_->item(row, 0)->setData(Qt::UserRole, it.key());
        table_->item(row, 6)->setForeground(QColor(it->online ? "#147052" : "#a72e38"));
    }
    if (row)
        table_->setCurrentCell(selectedRow, 0);
    const auto vehicle = vehicles.value(selectedKey());
    link_->setText(vehicle.online ? "Связь установлена" :
        row ? "Связь потеряна / нет HEARTBEAT" :
        manager_.isOpen() ? "Канал открыт · ожидание HEARTBEAT" : "Связь отсутствует");
    tone(link_, vehicle.online ? 1 : manager_.isOpen() && !row ? 0 : -1);
    open_->setEnabled(!manager_.isOpen());
    close_->setEnabled(manager_.isOpen());
    takeoff_->setEnabled(vehicle.online && !manager_.waiting());
    land_->setEnabled(takeoff_->isEnabled());
    sender_->setText(vehicle.port ? QString("Отправитель %1:%2  ·  Режим %3")
        .arg(vehicle.address.toString()).arg(vehicle.port).arg(vehicle.mode) : "Отправитель —");
    position_->setText(vehicle.positioned ? QString("Широта %1°  ·  Долгота %2°  ·  Высота %3 м")
        .arg(vehicle.latitude, 0, 'f', 7).arg(vehicle.longitude, 0, 'f', 7)
        .arg(vehicle.altitude, 0, 'f', 1) : "Телеметрия ожидается");
    series_->replace(vehicle.heights);
    const double latest = vehicle.heights.isEmpty() ? 0 : vehicle.heights.constLast().x();
    double top = 100;
    for (const auto &point : vehicle.heights)
        top = std::max(top, point.y() + 10);
    timeAxis_->setRange(std::max(0.0, latest - 30), std::max(30.0, latest));
    heightAxis_->setRange(0, top);

    QPainterPath path;
    if (!vehicle.route.isEmpty()) {
        path.moveTo(project(vehicle.route.constFirst()));
        for (const auto &point : vehicle.route)
            path.lineTo(project(point));
        marker_->setPos(project(vehicle.route.constLast()));
    }
    marker_->setVisible(vehicle.positioned);
    trail_->setPath(path);
}

void MainWindow::addEvent(const QString &type, const QString &message)
{
    journal_->insertItem(0, QString("%1  ·  %2  ·  %3")
        .arg(QTime::currentTime().toString("HH:mm:ss"), type, message));
    if (journal_->count() > 200)
        delete journal_->takeItem(200);
}
