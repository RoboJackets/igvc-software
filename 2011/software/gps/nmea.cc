#include "nmea.hpp"
#include <vector>
#include <boost/algorithm/string.hpp>

void nmea::decodeUTCTime(const std::string& val)
{

}
void nmea::decodeLatitude(const std::string& val)
{

}
void nmea::decodeLongitude(const std::string& val)
{

}

bool nmea::decodeGPRMC(const std::string& line)
{

}

bool nmea::decodeGPRMT(const std::string& line)
{

}

bool nmea::decodeGPGGA(const std::string& line)
{
	std::vector< std::string > splitvec;
	boost::algorithm::split(splitvec, line, boost::algorithm::is_any_of(","), boost::algorithm::token_compress_on);

	if(splitvec.size() != 15) return false;

	if(splitvec[0] != std::string("$GPGGA")) return false;

	decodeUTCTime(splitvec[1]);
	decodeLatitude(splitvec[2]);
	const char LatHemi = splitvec[3][0];
	decodeLongitude(splitvec[4]);
	const char LonHemi = splitvec[5][0];
	const char gpsQuality = splitvec[6][0];
	const std::string& numsat = splitvec[7];
	const std::string& horizDilutionPrec = splitvec[8];
	const std::string& sealevelheight = splitvec[9];
	const std::string& geoidalheight = splitvec[10];
	const std::string& diffgps = splitvec[11];
	const std::string& diffrefid = splitvec[12];
	return true;
}

bool nmea::decodeGPGSA(const std::string& line)
{

}

bool nmea::decodeGPGSV(const std::string& line)
{

}
