#include "positiontracker.h"
#include <cmath>
#include <common/config/configmanager.h>

PositionTracker::PositionTracker()
    : LOnGPSData(this),
      LOnMotionCommand(this),
      LOnIMUData(this)
{
}

Position PositionTracker::GetPosition()
{
    return _current_estimate;
}

Position PositionTracker::UpdateWithMeasurement(Position S, Position Measurement)
{
    Position result;
    result.Latitude.SetValue( ( S.Latitude.Value()*Measurement.Latitude.Variance() + Measurement.Latitude.Value()*S.Latitude.Variance() ) / (S.Latitude.Variance()+Measurement.Latitude.Variance()) );
    result.Longitude.SetValue( ( S.Longitude.Value()*Measurement.Longitude.Variance() + Measurement.Longitude.Value()*S.Longitude.Variance() ) / (S.Longitude.Variance()+Measurement.Longitude.Variance()) );
    result.Heading.SetValue( ( S.Heading.Value()*Measurement.Heading.Variance() + Measurement.Heading.Value()*S.Heading.Variance() ) / (S.Heading.Variance()+Measurement.Heading.Variance()) );

    result.Latitude.SetVariance( 1.0 / (1.0/S.Latitude.Variance() + 1.0 / Measurement.Latitude.Variance()) );
    result.Longitude.SetVariance( 1.0 / (1.0/S.Longitude.Variance() + 1.0 / Measurement.Longitude.Variance()) );
    result.Heading.SetVariance( 1.0 / (1.0/S.Heading.Variance() + 1.0 / Measurement.Heading.Variance()) );
    return result;
}

Position PositionTracker::UpdateWithMotion(Position S, Position Delta)
{
    Position result;
    result.Latitude.SetValue(S.Latitude.Value() + Delta.Latitude.Value());
    result.Longitude.SetValue(S.Longitude.Value() + Delta.Longitude.Value());
    result.Heading.SetValue(S.Heading.Value() + Delta.Heading.Value());

    result.Latitude.SetVariance(S.Latitude.Variance() + Delta.Latitude.Variance());
    result.Longitude.SetVariance(S.Longitude.Variance() + Delta.Longitude.Variance());
    result.Heading.SetVariance(S.Heading.Variance() + Delta.Heading.Variance());
    return result;
}

Position PositionTracker::DeltaFromMotionCommand(MotorCommand cmd)
{
    using namespace std;
    double V = ( cmd.rightVel + cmd.leftVel ) / 2.0;
    double W = ( cmd.rightVel - cmd.leftVel ) / ConfigManager::getValue("Robot", "Baseline", 1.0);
    double t = cmd.millis / 1000.0;
    double R = V/W;
    Position delta;
    double theta = _current_estimate.Heading.Value() * M_PI/180.0;
    double thetaPrime = (_current_estimate.Heading.Value() + W*t) * M_PI/180.0;
    delta.Heading.SetValue(W*t);
    delta.Latitude.SetValue(R*(cos(theta) - cos(thetaPrime)));
    delta.Longitude.SetValue(R*(sin(thetaPrime) - sin(theta)));
    // TODO - handle variances
    return delta;
}

void PositionTracker::OnGPSData(GPSData data)
{
    Position measurement;
    measurement.Latitude.SetValue(data.Lat());
    measurement.Longitude.SetValue(data.Long());
    // TODO - make sure the GPS actually outputs heading data
    measurement.Heading.SetValue(data.Heading());
    // TODO - handle variances
    _current_estimate = UpdateWithMeasurement(_current_estimate, measurement);
}

void PositionTracker::OnIMUData(IMUData data)
{

}

void PositionTracker::OnMotionCommand(MotorCommand cmd)
{
    _current_estimate = UpdateWithMotion(_current_estimate, DeltaFromMotionCommand(cmd));
}
