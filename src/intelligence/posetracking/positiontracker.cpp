#include "positiontracker.h"
#include <cmath>
#include <common/config/configmanager.h>

PositionTracker::PositionTracker()
    : LOnGPSData(this),
      LOnMotionCommand(this),
      LOnIMUData(this)
{
    _current_estimate.Latitude.Variance = 10000;
    _current_estimate.Longitude.Variance = 10000;
    _current_estimate.Heading.Variance = 10000;
}

Position PositionTracker::GetPosition()
{
    return _current_estimate;
}

Position PositionTracker::UpdateWithMeasurement(Position S, Position Measurement)
{
    Position result;
    result.Latitude = ( S.Latitude*Measurement.Latitude.Variance + Measurement.Latitude*S.Latitude.Variance ) / (S.Latitude.Variance+Measurement.Latitude.Variance);
    result.Longitude = ( S.Longitude*Measurement.Longitude.Variance + Measurement.Longitude*S.Longitude.Variance ) / (S.Longitude.Variance+Measurement.Longitude.Variance);
    result.Heading = ( S.Heading*Measurement.Heading.Variance + Measurement.Heading*S.Heading.Variance ) / (S.Heading.Variance+Measurement.Heading.Variance);

    result.Latitude.Variance = ( 1.0 / (1.0/S.Latitude.Variance + 1.0 / Measurement.Latitude.Variance) );
    result.Longitude.Variance = ( 1.0 / (1.0/S.Longitude.Variance + 1.0 / Measurement.Longitude.Variance) );
    result.Heading.Variance = ( 1.0 / (1.0/S.Heading.Variance + 1.0 / Measurement.Heading.Variance) );
    return result;
}

Position PositionTracker::UpdateWithMotion(Position S, Position Delta)
{
    Position result;
    result.Latitude = S.Latitude + Delta.Latitude;
    result.Longitude = S.Longitude + Delta.Longitude;
    result.Heading = S.Heading + Delta.Heading;

    result.Latitude.Variance = S.Latitude.Variance + Delta.Latitude.Variance;
    result.Longitude.Variance = S.Longitude.Variance + Delta.Longitude.Variance;
    result.Heading.Variance = S.Heading.Variance + Delta.Heading.Variance;
    return result;
}

Position PositionTracker::DeltaFromMotionCommand(MotorCommand cmd)
{
    /*
     * TODO - Need to make sure this accounts for the time it takes to actually get to where we're going.
     * Perhaps fire this after the motion command is done.
     */
    /*
     * TODO - This is doing meters only. We need to convert this translation into GPS coordinates
     */
    using namespace std;
    double V = ( cmd.rightVel + cmd.leftVel ) / 2.0;
    double W = ( cmd.rightVel - cmd.leftVel ) / ConfigManager::Instance().getValue("Robot", "Baseline", 1.0);
    double t = cmd.millis / 1000.0;
    double theta = _current_estimate.Heading * M_PI/180.0;
    Position delta;
    if(W != 0)
    {
        double R = V/W;
        double thetaPrime = (_current_estimate.Heading + W*t) * M_PI/180.0;
        delta.Heading = W*t;
        delta.Latitude = R*(cos(theta) - cos(thetaPrime));
        delta.Longitude = R*(sin(thetaPrime) - sin(theta));
    }
    else
    {
        delta.Heading = 0;
        delta.Latitude = V*t*cos(theta);
        delta.Longitude = V*t*sin(theta);
    }
    // TODO - handle variances
    return delta;
}

Position PositionTracker::MeasurementFromIMUData(IMUData data)
{
    // Uses the "Destination point given distance and bearing from start point" described here:
    // http://www.movable-type.co.uk/scripts/latlong.html#destPoint
    Position measurement;
    double lat1 = _current_estimate.Latitude;
    double lon1 = _current_estimate.Latitude;
    double hed1 = _current_estimate.Heading;
    double dx = data.X /* * time*time */;
    double dy = data.Y /* * time*time */;
    double d = sqrt(dx*dx + dy*dy); // Distance travelled
    double R = 6378137; // radius of Earth
    measurement.Latitude = asin(sin(lat1)*cos(d/R)+cos(lat1)*sin(d/R)*cos(hed1));
    measurement.Longitude = (lon1 + atan2(sin(hed1)*sin(d/R)*cos(lat1),cos(d/R)-sin(lat1)*sin(measurement.Latitude)));
    measurement.Heading = (atan2((measurement.Longitude-lon1)*sin(lat1), cos(measurement.Latitude)*sin(lat1)-sin(measurement.Latitude)*cos(lat1)*cos(measurement.Longitude-lon1)));
//    measurement.Heading = (measurement.Heading+180);
    while(measurement.Heading >= 360)
        measurement.Heading = (measurement.Heading - 360);
    while(measurement.Heading < 0)
        measurement.Heading = (measurement.Heading + 360);
    // TODO - handle variances
    return measurement;
}

void PositionTracker::OnGPSData(GPSData data)
{
    Position measurement;
    measurement.Latitude = data.Lat();
    measurement.Longitude = data.Long();
    // TODO - make sure the GPS actually outputs heading data
    measurement.Heading = data.Heading();

    measurement.Latitude.Variance = data.LatVar();
    measurement.Longitude.Variance = data.LongVar();
    measurement.Heading.Variance = data.HeadingVar();

    _current_estimate = UpdateWithMeasurement(_current_estimate, measurement);
}

void PositionTracker::OnIMUData(IMUData data)
{
    _current_estimate = UpdateWithMeasurement(_current_estimate, MeasurementFromIMUData(data));
}

void PositionTracker::OnMotionCommand(MotorCommand cmd)
{
    /*
     * TODO - Need to make sure this accounts for the time it takes to actually get to where we're going.
     * Perhaps fire this after the motion command is done.
     */
    _current_estimate = UpdateWithMotion(_current_estimate, DeltaFromMotionCommand(cmd));
}
