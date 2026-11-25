QT += network
CONFIG += c++17
SOURCES += mavlinkmessageparser.cpp telemetrymodel.cpp mavlinkmanager.cpp uavsimulator.cpp
HEADERS += mavlinkmessageparser.h telemetrymodel.h mavlinkmanager.h uavsimulator.h

# All three projects also support an in-source qmake build.
OBJECTS_DIR = .qmake/$$TARGET/obj
MOC_DIR = .qmake/$$TARGET/moc
UI_DIR = .qmake/$$TARGET/ui
