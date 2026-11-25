#pragma once
#include <QByteArray>
#include <QString>
#include <optional>

namespace Mavlink {
enum Message { Heartbeat = 0, Position = 33, CommandLong = 76, CommandAck = 77 };
constexpr quint16 Takeoff = 22, Land = 21;
constexpr quint8 StationSystem = 255, StationComponent = 190;
struct Frame {
    quint8 sequence, system, component;
    quint32 id;
    QByteArray payload;
};

std::optional<Frame> parse(const QByteArray &bytes, QString &error);
QByteArray encode(const Frame &frame);
quint8 byte(const QByteArray &bytes, int offset);
float readFloat(const QByteArray &bytes, int offset);
void writeFloat(QByteArray &bytes, int offset, float value);
}
