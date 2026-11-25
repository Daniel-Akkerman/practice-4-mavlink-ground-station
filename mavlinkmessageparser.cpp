#include "mavlinkmessageparser.h"
#include <QtEndian>
#include <cstring>

namespace Mavlink {
namespace {
struct Spec { int length; quint8 crcExtra; };
std::optional<Spec> spec(quint32 id)
{
    switch (id) {
    case Heartbeat: return Spec{9, 50};
    case Position: return Spec{28, 104};
    case CommandLong: return Spec{33, 152};
    case CommandAck: return Spec{10, 143};
    default: return std::nullopt;
    }
}

quint16 crc(const QByteArray &data, int end, quint8 extra)
{
    quint16 value = 0xffff;
    auto accumulate = [&value](quint8 next) {
        quint8 tmp = next ^ static_cast<quint8>(value);
        tmp ^= tmp << 4;
        value = (value >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    };
    for (int i = 1; i < end; ++i)
        accumulate(byte(data, i));
    accumulate(extra);
    return value;
}
}

quint8 byte(const QByteArray &bytes, int offset)
{
    return static_cast<quint8>(bytes.at(offset));
}

float readFloat(const QByteArray &bytes, int offset)
{
    const auto bits = qFromLittleEndian<quint32>(bytes.constData() + offset);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

void writeFloat(QByteArray &bytes, int offset, float value)
{
    quint32 bits;
    std::memcpy(&bits, &value, sizeof(bits));
    qToLittleEndian(bits, bytes.data() + offset);
}

std::optional<Frame> parse(const QByteArray &bytes, QString &error)
{
    if (bytes.size() < 12 || byte(bytes, 0) != 0xfd || byte(bytes, 2) != 0) {
        error = QStringLiteral("Повреждённый или неподдерживаемый заголовок MAVLink 2");
        return {};
    }
    const int length = byte(bytes, 1);
    const quint32 id = byte(bytes, 7) | (byte(bytes, 8) << 8) | (byte(bytes, 9) << 16);
    const auto format = spec(id);
    if (!format || length == 0 || length > format->length || bytes.size() != length + 12) {
        error = QStringLiteral("Неверная длина или неизвестный тип MAVLink-пакета");
        return {};
    }
    if (qFromLittleEndian<quint16>(bytes.constData() + 10 + length) !=
        crc(bytes, 10 + length, format->crcExtra)) {
        error = QStringLiteral("Неверная контрольная сумма CRC");
        return {};
    }
    QByteArray payload = bytes.mid(10, length);
    payload.resize(format->length, '\0');
    error.clear();
    return Frame{byte(bytes, 4), byte(bytes, 5), byte(bytes, 6), id, payload};
}

QByteArray encode(const Frame &frame)
{
    const auto format = spec(frame.id);
    if (!format || frame.payload.size() != format->length)
        return {};
    QByteArray payload = frame.payload;
    while (payload.size() > 1 && payload.endsWith('\0'))
        payload.chop(1);
    QByteArray bytes(10, '\0');
    bytes[0] = char(0xfd);
    bytes[1] = char(payload.size());
    bytes[4] = char(frame.sequence);
    bytes[5] = char(frame.system);
    bytes[6] = char(frame.component);
    bytes[7] = char(frame.id);
    bytes[8] = char(frame.id >> 8);
    bytes[9] = char(frame.id >> 16);
    bytes += payload;
    const auto checksum = crc(bytes, bytes.size(), format->crcExtra);
    bytes.append(char(checksum));
    bytes.append(char(checksum >> 8));
    return bytes;
}
}
