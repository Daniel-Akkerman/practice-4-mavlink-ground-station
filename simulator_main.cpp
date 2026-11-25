#include "uavsimulator.h"
#include <QApplication>
#include <QCheckBox>
#include <QGroupBox>
#include <QGridLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QWidget window;
    window.setWindowTitle("Практика №4 · Эмулятор БПЛА");
    auto *layout = new QVBoxLayout(&window);
    layout->addWidget(new QLabel("Телеметрия по UDP на 127.0.0.1:14540"));
    auto *log = new QListWidget;
    for (int id : {1, 2}) {
        auto *simulator = new UavSimulator(id, 14550 + id, &window);
        auto *group = new QGroupBox(QString("Борт %1 · UDP %2").arg(id).arg(14550 + id));
        auto *grid = new QGridLayout(group);
        auto *start = new QPushButton("Запустить");
        auto *stop = new QPushButton("Отключить БПЛА");
        auto *corrupt = new QPushButton("Повредить пакет");
        auto *skip = new QPushButton("Пропустить 2 пакета");
        auto *mute = new QCheckBox("Не отвечать ACK");
        auto *fault = new QCheckBox("Отказ сенсора");
        grid->addWidget(start, 0, 0);
        grid->addWidget(stop, 0, 1);
        grid->addWidget(corrupt, 1, 0);
        grid->addWidget(skip, 1, 1);
        grid->addWidget(mute, 2, 0);
        grid->addWidget(fault, 2, 1);
        layout->addWidget(group);
        QObject::connect(start, &QPushButton::clicked, simulator, [simulator] { simulator->start(); });
        QObject::connect(stop, &QPushButton::clicked, simulator, &UavSimulator::stop);
        QObject::connect(corrupt, &QPushButton::clicked, simulator, &UavSimulator::corruptNextPacket);
        QObject::connect(skip, &QPushButton::clicked, simulator, [simulator] { simulator->skipPackets(2); });
        QObject::connect(mute, &QCheckBox::toggled, simulator, [simulator](bool on) { simulator->setAckDrops(on ? -1 : 0); });
        QObject::connect(fault, &QCheckBox::toggled, simulator, &UavSimulator::setSensorFault);
        QObject::connect(simulator, &UavSimulator::eventLogged, log, [log](const QString &message) {
            log->insertItem(0, message);
            if (log->count() > 100)
                delete log->takeItem(100);
        });
    }
    layout->addWidget(log);
    window.resize(550, 425);
    window.show();
    return app.exec();
}
