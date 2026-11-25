#include "mainwindow.h"
#include "mavlinkmessageparser.h"
#include "uavsimulator.h"
#include <QDir>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QSignalSpy>
#include <QTableWidget>
#include <QtTest>

using namespace Mavlink;

class StationTests : public QObject {
    Q_OBJECT
private slots:
    void parser();
    void udpAndQuality();
    void commands();
    void windowScenario();
};

void StationTests::parser()
{
    // CRC 0x1ca3 cross-checked with bitwise CRC-16/MCRF4XX and CRC_EXTRA=50.
    const auto canonical = QByteArray::fromHex("fd090000000101000000000000000208000303a31c");
    QByteArray payload = QByteArray::fromHex("000000000208000303");
    const auto encoded = encode({0, 1, 1, Heartbeat, payload});
    QCOMPARE(encoded, canonical);
    QString error;
    QVERIFY(parse(canonical, error).has_value());
    QVERIFY(!parse(canonical.chopped(1), error));
    auto corrupt = canonical;
    corrupt[10] = 1;
    QVERIFY(!parse(corrupt, error));
    QVERIFY(error.contains("CRC"));
}

void StationTests::udpAndQuality()
{
    TelemetryModel model;
    MavlinkManager manager(model);
    UavSimulator first(1, 0), second(2, 0);
    QSignalSpy quality(&manager, &MavlinkManager::qualityChanged);
    QSignalSpy status(&manager, &MavlinkManager::commandStatus);
    manager.sendCommand(257, Takeoff, 80);
    QVERIFY(status.last().at(0).toString().contains("нет связи"));
    QVERIFY(!manager.isOpen());
    QVERIFY(manager.start(0));
    QVERIFY(first.start(manager.localPort()));
    QVERIFY(second.start(manager.localPort()));
    QTRY_COMPARE(model.vehicles().size(), 2);
    QTRY_VERIFY(model.vehicles().value(257).positioned && model.vehicles().value(513).positioned);
    QTRY_VERIFY(quality.last().at(3).toInt() >= 20);
    first.skipPackets(2);
    QTRY_COMPARE(quality.last().at(2).toULongLong(), 2ULL);
    first.corruptNextPacket();
    QTRY_COMPARE(quality.last().at(1).toULongLong(), 1ULL);
    QTRY_COMPARE(quality.last().at(2).toULongLong(), 3ULL);
    const auto count = quality.last().at(0).toULongLong();
    QTRY_VERIFY(quality.last().at(0).toULongLong() > count);
    second.stop();
    QTRY_VERIFY_WITH_TIMEOUT(!model.vehicles().value(513).online, 4200);
    QVERIFY(model.vehicles().value(257).online);
    QVERIFY(second.start(manager.localPort()));
    QTRY_VERIFY(model.vehicles().value(513).online);
    manager.stop();
    QVERIFY(model.vehicles().isEmpty());
}

void StationTests::commands()
{
    TelemetryModel model;
    MavlinkManager manager(model);
    UavSimulator uav(1, 0);
    QVERIFY(manager.start(0));
    QVERIFY(uav.start(manager.localPort()));
    QTRY_VERIFY(model.vehicles().value(257).online);
    QSignalSpy requests(&uav, &UavSimulator::commandReceived);
    QSignalSpy status(&manager, &MavlinkManager::commandStatus);
    uav.setAckDrops(1);
    manager.sendCommand(257, Takeoff, 80);
    QTRY_COMPARE_WITH_TIMEOUT(requests.size(), 2, 1600);
    QCOMPARE(requests.at(0).at(0).toInt(), int(Takeoff));
    QCOMPARE(requests.at(0).at(1).toFloat(), 80.0f);
    QCOMPARE(requests.at(0).at(2).toInt(), 0);
    QCOMPARE(requests.at(1).at(2).toInt(), 1);
    QTRY_VERIFY(!manager.waiting());
    QVERIFY(status.last().at(0).toString().contains("Команда принята"));
    QCOMPARE(uav.targetAltitude(), 80.0);
    uav.setSensorFault(true);
    QTRY_COMPARE(model.vehicles().value(257).mode, QString("Отказ сенсора"));
    manager.sendCommand(257, Land);
    QTRY_VERIFY(!manager.waiting());
    QVERIFY(status.last().at(0).toString().contains("Команда отклонена"));
    uav.setSensorFault(false);
    uav.setAckDrops(-1);
    requests.clear();
    manager.sendCommand(257, Takeoff, 90);
    QTRY_VERIFY_WITH_TIMEOUT(!manager.waiting(), 3700);
    QCOMPARE(requests.size(), 3);
    QCOMPARE(requests.last().at(2).toInt(), 2);
    QVERIFY(status.last().at(0).toString().contains("Подтверждение не получено"));
}

void StationTests::windowScenario()
{
    MainWindow window;
    window.show();
    auto *link = window.findChild<QLabel *>("labelLinkStatus");
    auto *fleet = window.findChild<QTableWidget *>("tableFleet");
    auto *takeoff = window.findChild<QPushButton *>("btnTakeoff");
    auto *start = window.findChild<QPushButton *>("btnStartLink");
    auto *height = window.findChild<QDoubleSpinBox *>("spinTakeoffAltitude");
    QVERIFY(link && fleet && takeoff && start && height);
    QCOMPARE(link->text(), QString("Связь отсутствует"));
    QVERIFY(!takeoff->isEnabled());
    QTest::mouseClick(start, Qt::LeftButton);
    QVERIFY(link->text().contains("Канал открыт"));
    UavSimulator first(1, 14551), second(2, 14552);
    QVERIFY(first.start());
    QVERIFY(second.start());
    QTRY_COMPARE(fleet->rowCount(), 2);
    QTRY_VERIFY(takeoff->isEnabled());
    height->setValue(80);
    QTest::mouseClick(takeoff, Qt::LeftButton);
    QTRY_COMPARE(first.targetAltitude(), 80.0);
    QTRY_VERIFY(takeoff->isEnabled());
    QTest::qWait(850);
    const auto output = qEnvironmentVariable("STATION_SCREENSHOT_DIR");
    if (!output.isEmpty()) {
        QVERIFY(QDir().mkpath(output));
        QVERIFY(window.grab().save(output + "/station-online.png"));
    }
    first.stop();
    QTRY_VERIFY_WITH_TIMEOUT(!takeoff->isEnabled(), 4200);
    QCOMPARE(fleet->item(0, 6)->text(), QString("Offline"));
    QCOMPARE(fleet->item(1, 6)->text(), QString("Online"));
    if (!output.isEmpty())
        QVERIFY(window.grab().save(output + "/station-offline.png"));
    QVERIFY(first.start());
    QTRY_VERIFY(takeoff->isEnabled());
}

QTEST_MAIN(StationTests)
#include "tests.moc"
