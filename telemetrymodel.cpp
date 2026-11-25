#include "telemetrymodel.h"

const QMap<int, VehicleTelemetry> &TelemetryModel::vehicles() const
{
    return vehicles_;
}

void TelemetryModel::setVehicle(int key, const VehicleTelemetry &vehicle)
{
    vehicles_.insert(key, vehicle);
    emit changed();
}

void TelemetryModel::clear()
{
    vehicles_.clear();
    emit changed();
}
