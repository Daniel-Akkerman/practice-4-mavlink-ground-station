#pragma once
#include "telemetrymodel.h"
#include <QElapsedTimer>
#include <QQueue>
#include <QTimer>
#include <QUdpSocket>
#include <optional>

class MavlinkManager : public QObject {
    Q_OBJECT
public:
    explicit MavlinkManager(TelemetryModel &model, QObject *parent = nullptr);
    bool start(quint16 port = 14540);
    void stop();
    bool isOpen() const;
    quint16 localPort() const;
    bool waiting() const;
    void sendCommand(int key, quint16 command, double altitude = 0);
signals:
    void channelChanged();
    void qualityChanged(quint64 received, quint64 errors, quint64 lost, int rate, qint64 age);
    void eventLogged(const QString &type, const QString &message);
    void commandStatus(const QString &message, int tone);
private:
    struct Pending {
        int key;
        quint16 command;
        float altitude;
        int attempts = 0;
        bool inProgress = false;
    };
    void readDatagrams();
    void checkTimeouts();
    void sendAttempt();
    void onAck(int key, const QByteArray &payload);
    void finish(const QString &message, int tone);
    void publishQuality();
    TelemetryModel &model_;
    QUdpSocket socket_;
    QTimer watchdog_, ackTimer_;
    QElapsedTimer clock_;
    QQueue<qint64> recent_;
    qint64 lastPacket_ = -1;
    quint64 received_ = 0, errors_ = 0, lost_ = 0;
    quint8 sequence_ = 0;
    std::optional<Pending> pending_;
};
