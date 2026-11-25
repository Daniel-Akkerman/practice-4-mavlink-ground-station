#include "uavsimulator.h"
#include "mavlinkmessageparser.h"
#include <QNetworkDatagram>
#include <QtEndian>
#include <algorithm>
#include <cmath>

using namespace Mavlink;

UavSimulator::UavSimulator(quint8 system, quint16 sourcePort, QObject *parent)
    : QObject(parent), system_(system), sourcePort_(sourcePort)
{
    timer_.setTimerType(Qt::PreciseTimer);
    connect(&timer_, &QTimer::timeout, this, &UavSimulator::transmit);
    connect(&socket_, &QUdpSocket::readyRead, this, &UavSimulator::readCommands);
}

bool UavSimulator::start(quint16 stationPort)
{
    if (timer_.isActive())
        return true;
    if (!socket_.bind(QHostAddress::LocalHost, sourcePort_, QUdpSocket::DontShareAddress)) {
        emit eventLogged(socket_.errorString());
        return false;
    }
    stationPort_ = stationPort;
    ticks_ = 0;
    timer_.start(100);
    transmit();
    emit eventLogged(QString("Борт %1 · UDP %2 · передача включена").arg(system_).arg(socket_.localPort()));
    return true;
}

void UavSimulator::stop()
{
    timer_.stop();
    socket_.close();
    emit eventLogged(QString("Борт %1 · передача остановлена").arg(system_));
}

void UavSimulator::corruptNextPacket() { corrupt_ = true; }
void UavSimulator::skipPackets(int count) { remainingSkips_ = count; }
void UavSimulator::setAckDrops(int count) { ackDrops_ = count; }
void UavSimulator::setSensorFault(bool fault) { fault_ = fault; }
double UavSimulator::targetAltitude() const { return targetAltitude_; }

void UavSimulator::send(quint32 id, const QByteArray &payload)
{
    auto bytes = encode({sequence_++, system_, 1, id, payload});
    if (remainingSkips_ > 0 && id != CommandAck) {
        --remainingSkips_;
        return;
    }
    if (corrupt_ && id == Position) {
        bytes[10] = bytes.at(10) ^ 1;
        corrupt_ = false;
    }
    socket_.writeDatagram(bytes, QHostAddress::LocalHost, stationPort_);
}

void UavSimulator::transmit()
{
    if (ticks_ % 10 == 0) {
        QByteArray heartbeat(9, '\0');
        qToLittleEndian<quint32>(guided_ ? 4 : 0, heartbeat.data());
        heartbeat[4] = 2;
        heartbeat[5] = 8;
        heartbeat[6] = guided_ ? 1 : 0;
        heartbeat[7] = fault_ ? 5 : guided_ ? 4 : 3;
        heartbeat[8] = 3;
        send(Heartbeat, heartbeat);
    }

    if (!fault_)
        altitude_ += std::clamp(targetAltitude_ - altitude_, -1.5, 1.5);
    const double phase = ticks_ * 0.045;
    const double latitude = 55.752220 + 0.00035 * std::sin(phase) + (system_ - 1) * 0.0001;
    const double longitude = 37.615560 + 0.00055 * std::cos(phase);
    QByteArray position(28, '\0');
    qToLittleEndian(ticks_++ * 100, position.data());
    qToLittleEndian(qint32(std::lround(latitude * 1e7)), position.data() + 4);
    qToLittleEndian(qint32(std::lround(longitude * 1e7)), position.data() + 8);
    qToLittleEndian(qint32(std::lround(altitude_ * 1000)), position.data() + 12);
    qToLittleEndian(qint32(std::lround(altitude_ * 1000)), position.data() + 16);
    send(Position, position);
}

void UavSimulator::readCommands()
{
    while (socket_.hasPendingDatagrams()) {
        const auto packet = socket_.receiveDatagram();
        if (packet.senderAddress() != QHostAddress::LocalHost || packet.senderPort() != stationPort_)
            continue;
        QString error;
        const auto frame = parse(packet.data(), error);
        if (!frame || frame->id != CommandLong)
            continue;
        const auto &data = frame->payload;
        if (byte(data, 30) != system_ || byte(data, 31) != 1)
            continue;
        const auto command = qFromLittleEndian<quint16>(data.constData() + 28);
        const float altitude = readFloat(data, 24);
        const int confirmation = byte(data, 32);
        emit commandReceived(command, altitude, confirmation);

        quint8 result = 0;
        if (command != Takeoff && command != Land)
            result = 3;
        else if (fault_ || (command == Takeoff && (!std::isfinite(altitude) || altitude < 10 || altitude > 500)))
            result = 2;
        else {
            targetAltitude_ = command == Takeoff ? altitude : 0;
            guided_ = true;
        }

        emit eventLogged(QString("Борт %1 · команда %2 · попытка %3 · %4").arg(system_)
            .arg(command).arg(confirmation + 1).arg(ackDrops_ != 0 ? "ACK пропущен" : "ACK отправлен"));
        if (ackDrops_ != 0) {
            if (ackDrops_ > 0)
                --ackDrops_;
            continue;
        }
        QByteArray ack(10, '\0');
        qToLittleEndian(command, ack.data());
        ack[2] = char(result);
        ack[8] = char(frame->system);
        ack[9] = char(frame->component);
        send(CommandAck, ack);
    }
}
