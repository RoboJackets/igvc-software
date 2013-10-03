#include "igvcsearchproblem.h"

using namespace pcl;

std::list<SearchMove> IGVCSearchProblem::getActions(SearchLocation state)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(Map.makeShared());
    //cout << "Expanding " << state << endl;
    std::list<SearchMove> acts;
    double delta = 0.001;
    double Wmin = -0.8;
    double Wmax =  0.8;
    for(double W = Wmin; W <= Wmax; W+=delta)
    {
        SearchMove move(Speed, W);
        SearchLocation result = getResult(state, move);
        pcl::PointXYZ searchPoint(result.x, result.y,0);
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
            SearchMove move = SearchMove(-Speed, W);
            SearchLocation result = getResult(state, move);
            pcl::PointXYZ searchPoint = pcl::PointXYZ(result.x, result.y,0);
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            if ( kdtree.radiusSearch(searchPoint, Threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0 )
            {
                acts.push_back(move);
            }
        }
    }
    if(PointTurnsEnabled){
        SearchMove move(0, TurningSpeed);
        SearchLocation result = getResult(state, move);
        pcl::PointXYZ searchPoint(result.x, result.y,0);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if ( kdtree.radiusSearch(searchPoint, Threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0 )
        {
            acts.push_back(move);
        }
        move = SearchMove(0, -TurningSpeed);
        result = getResult(state, move);
        searchPoint = pcl::PointXYZ(result.x, result.y,0);
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        if ( kdtree.radiusSearch(searchPoint, Threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0 )
        {
            acts.push_back(move);
        }
    }
    return acts;
}

SearchLocation IGVCSearchProblem::getResult(SearchLocation state, SearchMove action)
{
    SearchLocation result;
    if(action.W != 0)
    {
        double w = action.W;
        double R = action.V / action.W;
        double ICCx = state.x + cos(state.theta - M_PI/2.0) * R;
        double ICCy = state.y + sin(state.theta - M_PI/2.0) * R;
        using namespace Eigen;
        Matrix3d T;
        double wdt = w*DeltaT;
        T << cos(wdt), -sin(wdt), 0, sin(wdt), cos(wdt), 0, 0, 0, 1;
        Vector3d a(state.x - ICCx, state.y - ICCy, state.theta);
        Vector3d b(ICCx, ICCy, wdt);
        Vector3d c = T * a + b;
        result.x = c[0];
        result.y = c[1];
        result.theta = fmod(c[2], M_PI*2.0);
        result.theta = (result.theta > 0) ? result.theta : result.theta + M_PI*2.0;
    }
    else
    {
        result.theta = state.theta;
        result.x = state.x + cos(result.theta) * action.V * DeltaT;
        result.y = state.y + sin(result.theta) * action.V * DeltaT;
    }
    return result;
}
