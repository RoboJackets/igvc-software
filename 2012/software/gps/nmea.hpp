#ifndef NMEA
#define NMEA

#include "gps_common.hpp"

#include <string>

namespace nmea
{
	//GPS funcs
	bool decodeGPRMC(const std::string& line, GPSState& state);
	bool decodeGPRMT(const std::string& line);
	bool decodeGPGGA(const std::string& line, GPSState& state);
	bool decodeGPGSA(const std::string& line);
	bool decodeGPGSV(const std::string& line);

	//IMU funcs
	bool decodeRPY(const std::string& line, gyroState& state);

	void decodeUTCTime(const std::string& val);
	double decodeLatitude(const std::string& val, const char hemi);
	double decodeLongitude(const std::string& val, const char hemi);
}

#endif
