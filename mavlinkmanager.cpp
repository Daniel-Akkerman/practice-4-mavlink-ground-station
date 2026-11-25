#include "mavlinkmanager.h"
#include "mavlinkmessageparser.h"
#include <QNetworkDatagram>
#include <QtEndian>
#include <cmath>
#include <limits>

using namespace Mavlink;

MavlinkManager::MavlinkManager(TelemetryModel &model, QObject *parent)
    : QObject(parent), model_(model)
{
    connect(&socket_, &QUdpSocket::readyRead, this, &MavlinkManager::readDatagrams);
    connect(&watchdog_, &QTimer::timeout, this, &MavlinkManager::checkTimeouts);
    ackTimer_.setSingleShot(true);
    ackTimer_.setTimerType(Qt::PreciseTimer);
    connect(&ackTimer_, &QTimer::timeout, this, &MavlinkManager::sendAttempt);
}

bool MavlinkManager::isOpen() const { return socket_.state() == QAbstractSocket::BoundState; }
quint16 MavlinkManager::localPort() const { return socket_.localPort(); }
bool MavlinkManager::waiting() const { return pending_.has_value(); }

bool MavlinkManager::start(quint16 port)
{
    if (isOpen())
        return true;
    if (!socket_.bind(QHostAddress::LocalHost, port, QUdpSocket::DontShareAddress)) {
        emit eventLogged("UDP", socket_.errorString());
        return false;
    }
    clock_.start();
    watchdog_.start(100);
    emit channelChanged();
    emit eventLogged("UDP", QString("Открыт 127.0.0.1:%1").arg(localPort()));
    return true;
}

void MavlinkManager::stop()
{
    socket_.close();
    watchdog_.stop();
    if (pending_)
        finish("Канал закрыт", -1);
    model_.clear();
    recent_.clear();
    lastPacket_ = -1;
    received_ = errors_ = lost_ = 0;
    publishQuality();
    emit channelChanged();
    emit eventLogged("UDP", "Канал закрыт");
}

void MavlinkManager::publishQuality()
{
    const qint64 now = clock_.isValid() ? clock_.elapsed() : 0;
    while (!recent_.isEmpty() && recent_.head() <= now - 1000)
        recent_.dequeue();
    emit qualityChanged(received_, errors_, lost_, recent_.size(),
                        lastPacket_ < 0 ? -1 : now - lastPacket_);
}

void MavlinkManager::readDatagrams()
{
    while (socket_.hasPendingDatagrams()) {
        const auto packet = socket_.receiveDatagram();
        QString error;
        const auto frame = parse(packet.data(), error);
        if (!frame) {
            ++errors_;
            emit eventLogged("Пакет", error);
            continue;
        }
        ++received_;
        const qint64 now = clock_.elapsed();
        lastPacket_ = now;
        if (frame->id == Position || frame->id == Heartbeat)
            recent_.enqueue(now);
        const int key = (frame->system << 8) | frame->component;
        auto vehicle = model_.vehicles().value(key);
        if (vehicle.online &&
            (vehicle.port != packet.senderPort() || vehicle.address != packet.senderAddress())) {
            emit eventLogged("UDP", "Отклонён пакет с другого адреса или порта для занятого ID");
            continue;
        }
        if (vehicle.sequence >= 0) {
            const int advance = (frame->sequence - vehicle.sequence + 256) % 256;
            if (advance == 0 || advance >= 128)
                continue;
            if (advance > 1) {
                lost_ += advance - 1;
                emit eventLogged("seq", QString("Борт %1 · пропущено %2").arg(frame->system).arg(advance - 1));
            }
        }

        vehicle.system = frame->system;
        vehicle.component = frame->component;
        vehicle.sequence = frame->sequence;
        vehicle.address = packet.senderAddress();
        vehicle.port = packet.senderPort();
        vehicle.lastPacket = now;
        vehicle.received = QDateTime::currentDateTime();
        const auto &payload = frame->payload;
        if (frame->id == Heartbeat) {
            vehicle.heartbeat = now;
            if (!vehicle.online)
                emit eventLogged("Связь", QString("Борт %1 · связь установлена").arg(vehicle.system));
            vehicle.online = true;
            const auto state = byte(payload, 7);
            const QString mode = state == 5 ? "Отказ сенсора" :
                qFromLittleEndian<quint32>(payload.constData()) == 4 ? "GUIDED" : "STANDBY";
            if (mode != vehicle.mode)
                emit eventLogged("Режим", QString("Борт %1 · %2").arg(vehicle.system).arg(mode));
            vehicle.mode = mode;
        } else if (frame->id == Position) {
            vehicle.latitude = qFromLittleEndian<qint32>(payload.constData() + 4) / 1e7;
            vehicle.longitude = qFromLittleEndian<qint32>(payload.constData() + 8) / 1e7;
            vehicle.altitude = qFromLittleEndian<qint32>(payload.constData() + 12) / 1000.0;
            vehicle.positioned = true;
            vehicle.heights.append({now / 1000.0, vehicle.altitude});
            vehicle.route.append({vehicle.longitude, vehicle.latitude});
            if (vehicle.heights.size() > 300) {
                vehicle.heights.removeFirst();
                vehicle.route.removeFirst();
            }
        }
        model_.setVehicle(key, vehicle);
        if (frame->id == CommandAck)
            onAck(key, payload);
    }
    publishQuality();
}

void MavlinkManager::checkTimeouts()
{
    const auto snapshot = model_.vehicles();
    for (auto it = snapshot.cbegin(); it != snapshot.cend(); ++it) {
        if (!it->online || clock_.elapsed() - it->heartbeat <= 3000)
            continue;
        auto vehicle = it.value();
        vehicle.online = false;
        vehicle.sequence = -1;
        model_.setVehicle(it.key(), vehicle);
        emit eventLogged("Связь", QString("Борт %1 · связь потеряна: нет HEARTBEAT").arg(vehicle.system));
        if (pending_ && pending_->key == it.key())
            finish("Связь потеряна во время ожидания ACK", -1);
    }
    publishQuality();
}

void MavlinkManager::sendCommand(int key, quint16 command, double altitude)
{
    if (pending_)
        return;
    if (!isOpen() || !model_.vehicles().value(key).online) {
        finish("Команда недоступна: нет связи с бортом", -1);
        return;
    }
    if (command != Land && (command != Takeoff || !std::isfinite(altitude) || altitude < 10 || altitude > 500)) {
        finish("Недопустимая команда или высота", -1);
        return;
    }
    pending_ = Pending{key, command, float(altitude)};
    sendAttempt();
}

void MavlinkManager::sendAttempt()
{
    if (!pending_)
        return;
    auto &request = *pending_;
    if (request.inProgress || request.attempts >= 3) {
        finish("Подтверждение не получено", -1);
        return;
    }
    const auto vehicle = model_.vehicles().value(request.key);
    if (!vehicle.online || clock_.elapsed() - vehicle.heartbeat > 3000) {
        finish("Связь потеряна во время ожидания ACK", -1);
        return;
    }
    QByteArray payload(33, '\0');
    for (int offset : {12, 16, 20})
        writeFloat(payload, offset, std::numeric_limits<float>::quiet_NaN());
    writeFloat(payload, 24, request.altitude);
    qToLittleEndian(request.command, payload.data() + 28);
    payload[30] = char(vehicle.system);
    payload[31] = char(vehicle.component);
    payload[32] = char(request.attempts++);
    const auto bytes = encode({sequence_++, StationSystem, StationComponent, CommandLong, payload});
    if (socket_.writeDatagram(bytes, vehicle.address, vehicle.port) != bytes.size()) {
        finish("Ошибка отправки UDP", -1);
        return;
    }
    const auto status = QString("Борт %1 · команда %2 · попытка %3/3 · ожидание ACK")
        .arg(vehicle.system).arg(request.command).arg(request.attempts);
    ackTimer_.start(1000);
    emit eventLogged("Команда", status);
    emit commandStatus(status, 0);
}

void MavlinkManager::onAck(int key, const QByteArray &payload)
{
    if (!pending_ || pending_->key != key ||
        qFromLittleEndian<quint16>(payload.constData()) != pending_->command ||
        (byte(payload, 8) && byte(payload, 8) != StationSystem) ||
        (byte(payload, 9) && byte(payload, 9) != StationComponent))
        return;
    switch (byte(payload, 2)) {
    case 0: finish("Команда принята", 1); break;
    case 1: finish("Команда временно отклонена", -1); break;
    case 2: finish("Команда отклонена", -1); break;
    case 3: finish("Команда не поддерживается", -1); break;
    case 4: finish("Ошибка выполнения команды", -1); break;
    case 5:
        pending_->inProgress = true;
        ackTimer_.start(3000);
        emit commandStatus("Команда выполняется", 0);
        emit eventLogged("ACK", "Команда выполняется");
        break;
    case 6: finish("Команда отменена", -1); break;
    default: finish("Неизвестный результат ACK", -1); break;
    }
}

void MavlinkManager::finish(const QString &message, int tone)
{
    ackTimer_.stop();
    const auto details = pending_ ? QString("Борт %1 · команда %2 · %3")
        .arg(pending_->key >> 8).arg(pending_->command).arg(message) : message;
    pending_.reset();
    emit eventLogged("Команда", details);
    emit commandStatus(details, tone);
}
