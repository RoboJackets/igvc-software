#include "positiontracker.h"
#include <cmath>

PositionTracker::PositionTracker()
{
}

Position PositionTracker::UpdateWithMeasurement(Position S, Position Measurement)
{
    Position result;
    result.X.SetValue( ( S.X.Value()*Measurement.X.Variance() + Measurement.X.Value()*S.X.Variance() ) / (S.X.Variance()+Measurement.X.Variance()) );
    result.Y.SetValue( ( S.Y.Value()*Measurement.Y.Variance() + Measurement.Y.Value()*S.Y.Variance() ) / (S.Y.Variance()+Measurement.Y.Variance()) );
    result.T.SetValue( ( S.T.Value()*Measurement.T.Variance() + Measurement.T.Value()*S.T.Variance() ) / (S.T.Variance()+Measurement.T.Variance()) );

    result.X.SetVariance( 1.0 / (1.0/S.X.Variance() + 1.0 / Measurement.X.Variance()) );
    result.Y.SetVariance( 1.0 / (1.0/S.Y.Variance() + 1.0 / Measurement.Y.Variance()) );
    result.T.SetVariance( 1.0 / (1.0/S.T.Variance() + 1.0 / Measurement.T.Variance()) );
    return result;
}

Position PositionTracker::UpdateWithMotion(Position S, Position Delta)
{
    Position result;
    result.X.SetValue(S.X.Value() + Delta.X.Value());
    result.Y.SetValue(S.Y.Value() + Delta.Y.Value());
    result.T.SetValue(S.T.Value() + Delta.T.Value());

    result.X.SetVariance(S.X.Variance() + Delta.X.Variance());
    result.Y.SetVariance(S.Y.Variance() + Delta.Y.Variance());
    result.T.SetVariance(S.T.Variance() + Delta.T.Variance());
    return result;
}

Position PositionTracker::DelaFromMotionCommand()
{
    using namespace std;
    double V, W, t;
    double R = V/W;
    Position delta;
    double theta = _current_estimate.T.Value() * M_PI/180.0;
    double thetaPrime = (_current_estimate.T.Value() + W*t) * M_PI/180.0;
    delta.T.SetValue(W*t);
    delta.X.SetValue(R*(cos(theta) - cos(thetaPrime)));
    delta.Y.SetValue(R*(sin(thetaPrime) - sin(theta)));
    return delta;
}
