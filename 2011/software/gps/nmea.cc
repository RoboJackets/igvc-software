#include "nmea.hpp"
#include <vector>
#include <cmath>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

void nmea::decodeUTCTime(const std::string& val)
{

}
double nmea::decodeLatitude(const std::string& val, const char hemi)
{
	double raw = boost::lexical_cast<double>(val.c_str());
	double intpart;
	double frac = modf(raw / 100.0, &intpart);
	double deg = intpart + frac / 60.0;

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
	double deg = intpart + frac / 60.0;

	if(hemi == 'E')
	{
		return deg;
	}
	else
	{
		return deg * -1.0;
	}
}

bool nmea::decodeGPRMC(const std::string& line, GPSState& state)
{
	std::vector< std::string > splitvec;
	boost::algorithm::split(splitvec, line, boost::algorithm::is_any_of(","), boost::algorithm::token_compress_off);

	if(splitvec.size() != 13) return false;

	if(splitvec[0] != std::string("$GPRMC")) return false;

	decodeUTCTime(splitvec[1]);
	
	char status = splitvec[2][0];
	switch(status)
	{
		case 'A':
		{
			state.qual = GPS_QUALITY_NON_DIFF;
			break;
		}
		case 'V':
		{
			state.qual = GPS_QUALITY_NOFIX;
			break;
		}
		default:
		{
			state.qual = GPS_QUALITY_UNKNOWN;
			break;
		}
	}


	const char LatHemi = splitvec[4][0];
	state.lat = decodeLatitude(splitvec[3], LatHemi);

	const char LonHemi = splitvec[6][0];
	state.lon = decodeLongitude(splitvec[5], LonHemi);

	double speedKTS = boost::lexical_cast<double>(splitvec[7].c_str());

	state.courseoverground = boost::lexical_cast<double>(splitvec[8].c_str());

	std::string utcdate = splitvec[9];

	try
	{
		double magvariation = boost::lexical_cast<double>(splitvec[10].c_str());
	}
	catch(...)
	{
		double magvariation = 0;
	}

	char magvardir = splitvec[11][0];

	char mode = splitvec[12][0];

	return true;
}

bool nmea::decodeGPRMT(const std::string& line)
{
	return false;
}

bool nmea::decodeGPGGA(const std::string& line, GPSState& state)
{
	std::vector< std::string > splitvec;
	boost::algorithm::split(splitvec, line, boost::algorithm::is_any_of(","), boost::algorithm::token_compress_off);

	if(splitvec.size() != 15) return false;

	if(splitvec[0] != std::string("$GPGGA")) return false;

	decodeUTCTime(splitvec[1]);

	const char LatHemi = splitvec[3][0];
	state.lat = decodeLatitude(splitvec[2], LatHemi);

	const char LonHemi = splitvec[5][0];
	state.lon = decodeLongitude(splitvec[4], LonHemi);

	const char gpsQuality = splitvec[6][0];
	switch(gpsQuality)
	{
		case '0':
		{
			state.qual = GPS_QUALITY_NOFIX;
			break;
		}
		case '1':
		{
			state.qual = GPS_QUALITY_NON_DIFF;
			break;
		}
		case '2':
		{
			state.qual = GPS_QUALITY_WAAS;
			break;
		}
		case '6':
		{
			state.qual = GPS_QUALITY_ESTIMATED;
			break;
		}
		default:
		{
			state.qual = GPS_QUALITY_UNKNOWN;
			break;
		}
	}

	const std::string& numsat = splitvec[7];
	state.num_sat = boost::lexical_cast<int>(numsat.c_str());

	const std::string& horizDilutionPrec = splitvec[8];
	const std::string& sealevelheight = splitvec[9];
	const std::string& geoidalheight = splitvec[10];
	const std::string& diffgps = splitvec[11];
	const std::string& diffrefid = splitvec[12];
	return true;
}

bool nmea::decodeGPGSA(const std::string& line)
{
	return false;
}

bool nmea::decodeGPGSV(const std::string& line)
{
	return false;
}
