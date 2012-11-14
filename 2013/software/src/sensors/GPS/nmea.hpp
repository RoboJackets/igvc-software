#ifndef NMEA
#define NMEA

#include "GPS.hpp"

#include <string>

using namespace IGVC::Sensors;

namespace nmea
{
	//GPS funcs
	bool decodeGPRMC(const std::string& line, GPSState& state);
	bool decodeGPRMT(const std::string& line);
	bool decodeGPGGA(const std::string& line, GPSState& state);
	bool decodeGPGSA(const std::string& line);
	bool decodeGPGSV(const std::string& line);
	void decodeUTCTime(const std::string& val);
	double decodeLatitude(const std::string& val, const char hemi);
	double decodeLongitude(const std::string& val, const char hemi);
}

#endif
