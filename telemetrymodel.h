#pragma once
#include <QDateTime>
#include <QHostAddress>
#include <QMap>
#include <QObject>
#include <QPointF>
#include <QVector>

struct VehicleTelemetry {
    quint8 system = 0, component = 0;
    QHostAddress address;
    quint16 port = 0;
    QDateTime received;
    qint64 lastPacket = -1, heartbeat = -1;
    int sequence = -1;
    bool online = false, positioned = false;
    double latitude = 0, longitude = 0, altitude = 0;
    QString mode = QStringLiteral("—");
    QVector<QPointF> heights, route;
};

class TelemetryModel : public QObject {
    Q_OBJECT
public:
    using QObject::QObject;
    const QMap<int, VehicleTelemetry> &vehicles() const;
    void setVehicle(int key, const VehicleTelemetry &vehicle);
    void clear();
signals:
    void changed();
private:
    QMap<int, VehicleTelemetry> vehicles_;
};
