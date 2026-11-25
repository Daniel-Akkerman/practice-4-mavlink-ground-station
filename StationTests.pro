TEMPLATE = app
TARGET = StationTests
QT += widgets charts testlib
CONFIG += testcase console
include(station-network.pri)
include(station-window.pri)
SOURCES += tests.cpp
