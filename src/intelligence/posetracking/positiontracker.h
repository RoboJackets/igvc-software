#ifndef POSITIONTRACKER_H
#define POSITIONTRACKER_H

#include <gaussianvariable.hpp>

class Position
{
    // Global horizontal component
    GaussianVariable<double> X;
    // Global vertical component
    GaussianVariable<double> Y;
    // Heading west of north (degrees)
    GaussianVariable<double> T;

    friend std::ostream &operator<< (std::ostream &stream, State &s)
    {
        stream << s.X.Value() << " (" << s.X.Variance() << "), ";
        stream << s.Y.Value() << " (" << s.Y.Variance() << "), ";
        stream << s.T.Value() << " (" << s.T.Variance() << ")";
        return stream;
    }
};

class PositionTracker
{
public:
    PositionTracker();

private:
    Position UpdateWithMotion(Position s, Position delta);
    Position UpdateWithMeasurement(Position s, Position measurement);

    Position DelaFromMotionCommand();

    Position _current_estimate;

};

#endif // POSITIONTRACKER_H
