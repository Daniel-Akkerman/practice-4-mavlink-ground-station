#pragma once
#include <QObject>
#include <QTimer>
#include <QUdpSocket>

class UavSimulator : public QObject {
    Q_OBJECT
public:
    UavSimulator(quint8 system, quint16 sourcePort, QObject *parent = nullptr);
    bool start(quint16 stationPort = 14540);
    void stop();
    void corruptNextPacket();
    void skipPackets(int count);
    void setAckDrops(int count);
    void setSensorFault(bool fault);
    double targetAltitude() const;
signals:
    void eventLogged(const QString &message);
    void commandReceived(quint16 command, float altitude, int confirmation);
private:
    void transmit();
    void readCommands();
    void send(quint32 id, const QByteArray &payload);
    QUdpSocket socket_;
    QTimer timer_;
    quint8 system_, sequence_ = 0;
    quint16 sourcePort_, stationPort_ = 0;
    quint32 ticks_ = 0;
    double altitude_ = 0, targetAltitude_ = 0;
    int remainingSkips_ = 0, ackDrops_ = 0;
    bool corrupt_ = false, fault_ = false, guided_ = false;
};
