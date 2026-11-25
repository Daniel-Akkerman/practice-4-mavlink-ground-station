TEMPLATE = app
TARGET = GroundStation
QT += widgets charts
include(station-network.pri)
include(station-window.pri)
SOURCES += main.cpp
