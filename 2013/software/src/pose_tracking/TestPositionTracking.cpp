#include <iostream>
#include <vector>
#include <sensors/GPS/HemisphereA100GPS.h>

//GPSAccuracy GPSData::NAV200Default = GPSAccuracy(.0001, .0001, 3, 0.01);
//GPSAccuracy IMUData::ArduPilotDefault= GPSAccuracy(.001, .001, 1, .1);

using namespace IGVC::Sensors;
using namespace std;

class GPSListener
{
private:
    GPSData _start;
    bool isFirst;
    vector<GPSData> _recents;
public:
    GPSListener(GPS* gps)
        : LonNewStateAvailable(this)
    {
        gps->onNewData += &LonNewStateAvailable;
        isFirst = true;
    }

    double distance(double lat1, double lon1, double lat2, double lon2)
    {
        double R = 6371000;
//        cout << setprecision(15) << lat2 << "  " << lat1 << "   " << lat2-lat1 << endl;
        double dLat = abs(lat2-lat1) / 180.0 * M_PI;
        double dLon = abs(lon2-lon1) / 180.0 * M_PI;
        lat1 = lat1 / 180.0 * M_PI;
        lat2 = lat2 / 180.0 * M_PI;

        double a = sin(dLat / 2.0) * sin(dLat / 2.0) + sin(dLon / 2.0) * sin(dLon / 2.0) * cos(lat1) * cos(lat2);
        double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));
        return R * c ;
    }

    void onNewStateAvailable(GPSData state)
    {
        if(state.Quality() == GPS_QUALITY_WAAS)
        {
            if(isFirst)
            {
                _start = state;
                isFirst = false;
            }

            double dist = distance(_start.Lat(), _start.Long(), state.Lat(), state.Long());

            cout << setprecision(15) << dist << endl;
        }
 /*       int NUMPOINTS =10;
        _recents.push_back(state);
        if(_recents.size() > NUMPOINTS)
        {
            _recents.erase(_recents.begin());
        }

        double lat = 0;
        double lon = 0;
        for(int i = 0; i < _recents.size(); i++)
        {
            GPSData d = _recents[i];
            lat += d.Lat();
            lon += d.Long();
        }
        if(_recents.size() > 0)
        {
            lat /= _recents.size();
            lon /= _recents.size();
        }

        if(_recents.size() >= NUMPOINTS)
        {
            if(isFirst)
            {
                _start.Lat(lat);
                _start.Long(lon);
                isFirst = false;
            }


            double dist = distance(_start.Lat(), _start.Long(), lat, lon);
            double dist2 = distance(_start.Lat(), _start.Long(), state.Lat(), state.Long());

//            double dist = distance(_start.Lat(), _start.Long(), _tracker->Lat()[0].value(), _tracker->Long()[0].value());
//            double dist2 = distance(_start.Lat(), _start.Long(), state.Lat(), state.Long());
//            cout << " Lat: " << lat << "  Lon: " << lon << endl <<"rLat: " << state.Lat() << " rLon: " << state.Long() << endl;
            cout << std::setprecision(15) << "Distance think : " << dist << " Distance raw : " << dist2 << endl;
//            cout << setprecision(15) << state.Lat() << endl << endl;
        }
*/
    }

    LISTENER(GPSListener, onNewStateAvailable, GPSData);

    ~GPSListener()
    {
    }
};

int main()
{

    HemisphereA100GPS gps;

    GPSListener gpsListener(&gps);

    while(true);

}
