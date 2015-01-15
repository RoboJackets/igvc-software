#include "nmea.hpp"
#include <vector>
#include <cmath>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>

void nmea::decodeUTCTime(const std::string&)
{
    // FIXME : unimplemented decodeUTCTime
}

double nmea::decodeLatitude(const std::string& val, const char hemi)
{
	double raw = boost::lexical_cast<double>(val.c_str());
	double intpart;
	double frac = modf(raw / 100.0, &intpart);
	double deg = intpart + frac * 100.0 / 60.0;

	if(hemi == 'N')
	{
		return deg;
	}
	else
	{
		return deg * -1.0;
	}
}

double nmea::decodeLongitude(const std::string& val, const char hemi)
{
	double raw = boost::lexical_cast<double>(val.c_str());
	double intpart;
	double frac = modf(raw / 100.0, &intpart);
	double deg = intpart + frac * 100.0 / 60.0;

	if(hemi == 'E')
	{
		return deg;
	}
	else
	{
		return deg * -1.0;
	}
}

void nmea::decodeGPRMC(const std::string& line, GPSData& state)
{
    try {
        std::vector< std::string > splitvec;
        boost::algorithm::split(splitvec, line, boost::algorithm::is_any_of(","), boost::algorithm::token_compress_off);


        if(splitvec.size() != 14)
            throw bad_nmea_sentence("RMC", line);

        if(splitvec[0] != std::string("$GPRMC"))
            throw bad_nmea_sentence("RMC", line);

        decodeUTCTime(splitvec[1]);

        char status = splitvec[2][0];
        switch(status)
        {
            case 'A':
            {
                state.Quality(GPS_QUALITY_SPS);
                break;
            }
            case 'V':
            {
                state.Quality(GPS_QUALITY_INVALID);
                break;
            }
            default:
            {
                state.Quality(GPS_QUALITY_INVALID);
                break;
            }
        }

        const char LatHemi = splitvec[4][0];
        state.Lat(decodeLatitude(splitvec[3], LatHemi));

        const char LonHemi = splitvec[6][0];
        state.Long(decodeLongitude(splitvec[5], LonHemi));

        double speedKTS = boost::lexical_cast<double>(splitvec[7].c_str());
        double speedms = speedKTS  * .514444444444444444;
        state.Speed(speedms);

        state.Heading(boost::lexical_cast<double>(splitvec[8].c_str()));

    //	std::string utcdate = splitvec[9];

    //	try
    //	{
    //		double magvariation = boost::lexical_cast<double>(splitvec[10].c_str());
    //	}
    //	catch(...)
    //	{
    //		double magvariation = 0;
    //	}

    //	char magvardir = splitvec[11][0];

    //	char mode = splitvec[12][0];
    } catch(boost::bad_lexical_cast) {
        throw bad_nmea_sentence("RMC", line);
    }
}

void nmea::decodeGPGGA(const std::string& line, GPSData& state)
{
    try {
        std::vector< std::string > splitvec;
        boost::algorithm::split(splitvec, line, boost::algorithm::is_any_of(","), boost::algorithm::token_compress_off);

        if(splitvec.size() != 15)
            throw bad_nmea_sentence("GGA", line);

        if(splitvec[0] != std::string("$GPGGA"))
            throw bad_nmea_sentence("GGA", line);

        decodeUTCTime(splitvec[1]);

        const char LatHemi = splitvec[3][0];
        state.Lat(decodeLatitude(splitvec[2], LatHemi));

        const char LonHemi = splitvec[5][0];
        state.Long(decodeLongitude(splitvec[4], LonHemi));

        state.Quality((GPS_QUALITY)boost::lexical_cast<int>(splitvec[6][0]));

        state.NumSats(boost::lexical_cast<int>(splitvec[7].c_str()));

        state.HDOP(boost::lexical_cast<float>(splitvec[8].c_str()));

        state.LatVar(1.1005e-11 * state.HDOP() - 7.6854e-12 );
        state.LongVar(5.5776e-10 * state.HDOP() - -3.9038e-10);

    //	const std::string& sealevelheight = splitvec[9];
    //	const std::string& geoidalheight = splitvec[10];
    //	const std::string& diffgps = splitvec[11];
    //	const std::string& diffrefid = splitvec[12];
    } catch (boost::bad_lexical_cast) {
        throw bad_nmea_sentence("GGA", line);
    }
}
