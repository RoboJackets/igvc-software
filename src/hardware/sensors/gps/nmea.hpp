#ifndef NMEA
#define NMEA

#include "GPS.hpp"

#include <string>

/*!
 * \brief Contains helper methods for interpreting messages received from devices following the NMEA GPS standard.
 */
namespace nmea
{
	//GPS funcs
	bool decodeGPRMC(const std::string& line, GPSData& state);
    /*! \note Not implemented */
    bool decodeGPRMT(const std::string&);
	bool decodeGPGGA(const std::string& line, GPSData& state);
    /*! \note Not implemented */
    bool decodeGPGSA(const std::string&);
    /*! \note Not implemented */
    bool decodeGPGSV(const std::string&);
    /*! \note Not implemented */
    void decodeUTCTime(const std::string&);
	double decodeLatitude(const std::string& val, const char hemi);
	double decodeLongitude(const std::string& val, const char hemi);
}

#endif
