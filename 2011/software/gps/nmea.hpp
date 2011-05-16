
#include <string>

namespace nmea
{
	bool decodeGPRMC(const std::string& line);
	bool decodeGPRMT(const std::string& line);
	bool decodeGPGGA(const std::string& line);
	bool decodeGPGSA(const std::string& line);
	bool decodeGPGSV(const std::string& line);

	void decodeUTCTime(const std::string& val);
	void decodeLatitude(const std::string& val);
	void decodeLongitude(const std::string& val);
}
