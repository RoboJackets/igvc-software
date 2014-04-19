#include "igvcsearchproblem.h"

using namespace pcl;

std::list<SearchMove> IGVCSearchProblem::getActions(SearchLocation state)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(Map.makeShared());
    //cout << "Expanding " << state << endl;
    std::list<SearchMove> acts;
    double delta = 0.05;
    double Wmin = -0.8;
    double Wmax =  0.8;
    for(double W = Wmin; W <= Wmax; W+=delta)
    {
        SearchMove move(Speed, W, DeltaT);
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
            SearchMove move = SearchMove(-Speed, W, DeltaT);
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
        SearchMove move(0, TurningSpeed, DeltaT);
        SearchLocation result = getResult(state, move);
        pcl::PointXYZ searchPoint(result.x, result.y,0);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if ( kdtree.radiusSearch(searchPoint, Threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0 )
        {
            acts.push_back(move);
        }
        move = SearchMove(0, -TurningSpeed, DeltaT);
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
    if(abs(action.W) > 1e-10)
    {
        double w = action.W;
        double R = action.V / action.W;
        double ICCx = state.x - ( R * cos(M_PI - state.theta) );
        double ICCy = state.y - ( R * sin(M_PI - state.theta) );
        using namespace Eigen;
        Matrix3d T;
        double wdt = w*DeltaT;
        T << cos(wdt), sin(wdt), 0, -sin(wdt), cos(wdt), 0, 0, 0, 1;
        Vector3d a(state.x - ICCx, state.y - ICCy, state.theta);
        Vector3d b = T*a;
        Vector3d c = b + Vector3d(ICCx, ICCy, wdt);
        //Vector3d b(ICCx, ICCy, wdt);
        //Vector3d c = T * a + b;
        result.x = c[0];
        result.y = c[1];
        result.theta = c[2];
        while(result.theta < 0)
            result.theta += 2*M_PI;
        while(result.theta > 2*M_PI)
            result.theta -= 2*M_PI;
    }
    else
    {
        result.theta = state.theta;
        result.x = state.x + ( cos(M_PI_2 - result.theta) * action.V * DeltaT );
        result.y = state.y + ( sin(M_PI_2 - result.theta) * action.V * DeltaT );
    }
    return result;
}
