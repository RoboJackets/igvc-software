#include <iostream>
//#include "timing.h"
#include "SensorData.h"
#include "DataArray.h"
//#include <boost/circular_buffer.hpp>

class GPSData : public SensorData
{
public:
    inline GPSData(double Latitude, double Longitude): SensorData(), Lati(Latitude), Longi(Longitude)
    {
    }

    inline double Lat(void)
    {
        return Lati;
    }

    inline double Long(void)
    {
        return Longi;
    }

    private:
        double Lati;
        double Longi;
};

int main()
{
    //GPSData dat(49.12, 0.2122);
    //std::cout << "Longitude is "<< dat.Long() << " Latitude is "<< dat.Lat() << " Time is " << dat.time() << std::endl;

    DataArray<GPSData> GPSArray(100);
    //GPSData dat(49.12, 0.2122);
    //GPSArray.push(dat);
    GPSArray.push(GPSData(49.12, 0.2122));
    std::cout << GPSArray[0].Lat() <<  " Should be 49.12"<< std::endl;
    std::cout << GPSArray.size() << std::endl;

}
