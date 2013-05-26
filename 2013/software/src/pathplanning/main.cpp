#include <iostream>
#include <GraphSearch.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

using namespace std;
using namespace pcl;

class Location
{
public:
    double x, y, theta;
    static const double sameness_threshold = 0.1;
    Location(){ }
    Location(double _x, double _y, double _theta)
    {
        x = _x;
        y = _y;
        theta = _theta;
    }
    friend ostream &operator<< (ostream &stream, Location &loc)
    {
        stream << "(" << loc.x << "," << loc.y << "," << loc.theta << ")";
        return stream;
    }
    bool operator == (const Location &other)
    {
        return abs(x - other.x) < sameness_threshold && abs(y - other.y) < sameness_threshold;// && abs(theta - other.theta) < sameness_threshold;
    }
    bool operator < (const Location &other) const
    {
        if(x < other.x)
        {
            return true;
        }
        else if(y < other.y)
        {
            return true;
        }
        else if(theta < other.theta)
        {
            return true;
        }
        return false;
    }
    double distTo(Location other)
    {
        return sqrt(pow(other.x - x, 2) + pow(other.y - y, 2));
    }
};

class Move
{
public:
    double V, W;
    Move() { }
    Move(double v, double w)
    {
        V = v;
        W = w;
    }
    friend ostream &operator<< (ostream &stream, Move &move)
    {
        stream << "(<" << move.V << "," << move.W << ">";
        return stream;
    }
    bool operator == (const Move &other)
    {
        return (V == other.V) && (W == other.W);
    }
};

class Problem : public SearchProblem<Location, Move>
{
public:

    PointCloud<PointXYZ> Map;
    Location Start;
    Location Goal;
    double Threshold;
    double Speed;
    double TurningSpeed;
    double DeltaT;
    double Baseline;
    bool PointTurnsEnabled;

    Location getStartState()
    {
        return Start;
    }
    std::list<Move> getActions(Location state)
    {
        KdTreeFLANN<PointXYZ> kdtree;
        kdtree.setInputCloud(Map.makeShared());
        //cout << "Expanding " << state << endl;
        std::list<Move> acts;
        double delta = 0.01;
        double Wmin = -M_PI;
        double Wmax =  M_PI;
        for(double W = Wmin; W <= Wmax; W+=delta)
        {
            Move move(Speed, W);
            Location result = getResult(state, move);
            PointXYZ searchPoint(result.x, result.y,0);
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            if ( kdtree.radiusSearch(searchPoint, Threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0 )
            {
                acts.push_back(move);
            }
        }
        if(acts.size() == 0)
        {
            for(double W = Wmin; W <= Wmax; W+=delta)
            {
                Move move = Move(-Speed, W);
                Location result = getResult(state, move);
                PointXYZ searchPoint = PointXYZ(result.x, result.y,0);
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;
                if ( kdtree.radiusSearch(searchPoint, Threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0 )
                {
                    acts.push_back(move);
                }
            }
        }
        if(PointTurnsEnabled){
            Move move(0, TurningSpeed);
            Location result = getResult(state, move);
            PointXYZ searchPoint(result.x, result.y,0);
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            if ( kdtree.radiusSearch(searchPoint, Threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0 )
            {
                acts.push_back(move);
            }
            move = Move(0, -TurningSpeed);
            result = getResult(state, move);
            searchPoint = PointXYZ(result.x, result.y,0);
            pointIdxRadiusSearch.clear();
            pointRadiusSquaredDistance.clear();
            if ( kdtree.radiusSearch(searchPoint, Threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0 )
            {
                acts.push_back(move);
            }
        }
        return acts;
    }
    Location getResult(Location state, Move action)
    {
        Location result;
        double Vr = action.V - (Baseline/2.0)*action.W;
        double Vl = action.V + (Baseline/2.0)*action.W;
        if(Vr - Vl != 0)
        {
            double w = -(Vr - Vl) / Baseline;
            double R = -(Baseline/2.0)*((Vl + Vr) / (Vr - Vl));
            double ICCx = state.x + cos(state.theta - M_PI/2.0) * R;
            double ICCy = state.y + sin(state.theta - M_PI/2.0) * R;
            using namespace Eigen;
            Matrix3d T;
            double wdt = w*DeltaT;
            T << cos(wdt), -sin(wdt), 0, sin(wdt), cos(wdt), 0, 0, 0, 1;
            Vector3d a(state.x - ICCx, state.y - ICCy, state.theta);
            Vector3d b(ICCx, ICCy, wdt);
            Vector3d c = T * a + b;
            //cout << c << endl;
            result.x = c[0];
            result.y = c[1];
            result.theta = fmod(c[2], M_PI*2.0);
            result.theta = (result.theta > 0) ? result.theta : result.theta + M_PI*2.0;
        }
        else
        {
            result.theta = state.theta;
            result.x = state.x + cos(result.theta) * Vl * DeltaT;
            result.y = state.y + sin(result.theta) * Vl * DeltaT;
        }
        return result;

    }
    bool isGoal(Location state)
    {
        return state == Goal;//state.distTo(Goal) < Threshold;
    }
    double getStepCost(Location state, Move action)
    {
        double R = abs(action.V) / abs(action.W);
        double theta = abs(action.W) * DeltaT;
        return R*theta;
    }
    double getHeuristicCost(Location state)
    {
        return state.distTo(Goal);
    }
};

int main()
{
    Problem problem;

    {
        double mapdelta = 0.25;
        for(double x = 0; x < 10; x+=mapdelta)
        {
            problem.Map.points.push_back(PointXYZ(x,0,0));
            problem.Map.points.push_back(PointXYZ(x,10-mapdelta,0));
        }
        for(double y = mapdelta; y < 10-mapdelta; y+=mapdelta)
        {
            problem.Map.points.push_back(PointXYZ(0, y, 0));
            problem.Map.points.push_back(PointXYZ(10-mapdelta, y, 0));
        }
        for(double y = mapdelta; y < 5; y+=mapdelta)
        {
            problem.Map.points.push_back(PointXYZ(2, y, 0));
        }
        for(double y = 10-mapdelta; y >= 4; y-=mapdelta)
        {
            problem.Map.points.push_back(PointXYZ(3.5,y,0));
        }
        for(double x = 3.5+mapdelta; x <= 7; x+=mapdelta)
        {
            problem.Map.points.push_back(PointXYZ(x,4,0));
        }
    }


    problem.Threshold = 0.5;
    problem.Start = Location(1,1,-M_PI/2.0);
    problem.Goal = Location(5,5,0);
    problem.Speed = 0.4;
    problem.DeltaT = 0.4;
    problem.Baseline = 0.71755;
    problem.PointTurnsEnabled = false;

    Path<Location, Move> path = GraphSearch::AStar(problem);

    cout << "Found path of length " << path.getNumberOfSteps() << ":" << endl;

    visualization::PCLVisualizer viewer("Map and Path");
    viewer.addSphere(PointXYZ(problem.Start.x, problem.Start.y,0), 0.1, 0, 1, 0, "start");
    viewer.addSphere(PointXYZ(problem.Goal.x, problem.Goal.y, 0),  0.1, 1, 0, 0, "end");
//    viewer.addLine(PointXYZ(1,1,0), PointXYZ(1+cos(problem.Start.theta)*0.1,1+sin(problem.Start.theta)*0.1,0),0,1,0);
    viewer.addPointCloud(problem.Map.makeShared());

    list<Location>* locations = path.getStates();
    list<Move>* moves = path.getActions();

    cout << "Locations along path:" << endl;
    Location lastLoc;
    int i=0;
    list<Move>::iterator act_iter = moves->begin();
    for(list<Location>::iterator it = locations->begin(); it != locations->end(); it++)
    {
        Location loc = *it;
        cout << loc << ", ";
        stringstream name;

        Move m = (*act_iter);
        if(m.W != 0)
        {
            name << "Sphere" << i;
            viewer.addSphere(PointXYZ(loc.x, loc.y, 0),  0.01, 1, 0, 1, name.str());
            name.clear();
            name << "Action" << i;
            double R = m.V/m.W;
            double ICCx = loc.x + cos(loc.theta - M_PI/2.0) * R;
            double ICCy = loc.y + sin(loc.theta - M_PI/2.0) * R;
            viewer.addLine<PointXYZ, PointXYZ>(PointXYZ(loc.x,loc.y,0), PointXYZ(ICCx,ICCy,0), 0,1,1,name.str());
        }
        if(m.V == 0)
        {
            name.clear();
            name << "Sphere" << i;
            viewer.addSphere(PointXYZ(loc.x, loc.y, 0),  0.01, 0, 0, 1, name.str());
        }

        if(i > 0)
        {
            name.clear();
            name << "Line" << i;
            if(m.V > 0)
                viewer.addLine<PointXYZ, PointXYZ>(PointXYZ(lastLoc.x, lastLoc.y,0), PointXYZ(loc.x, loc.y,0),0,1,0, name.str());
            else
                viewer.addLine<PointXYZ, PointXYZ>(PointXYZ(lastLoc.x, lastLoc.y,0), PointXYZ(loc.x, loc.y,0),1,0,0, name.str());
        }
        lastLoc = loc;
        i++;
        act_iter++;
    }
    cout << endl << endl;

    cout << "Actions required: " << endl;
    for(list<Move>::iterator it = moves->begin(); it != moves->end(); it++)
    {
        Move m = *it;
        cout << m << ", ";
    }
    cout << endl;

    while(!viewer.wasStopped())
    {
        viewer.spin();
    }

    return 0;
}
