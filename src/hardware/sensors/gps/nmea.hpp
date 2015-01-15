#ifndef NMEA
#define NMEA

#include "GPS.hpp"

#include <string>

/*!
 * \brief Contains helper methods for interpreting messages received from devices following the NMEA GPS standard.
 */
namespace nmea
{
    void decodeUTCTime(const std::string&);
    void decodeGPRMC(const std::string& line, GPSData& state);
    void decodeGPGGA(const std::string& line, GPSData& state);
	double decodeLatitude(const std::string& val, const char hemi);
	double decodeLongitude(const std::string& val, const char hemi);

    class bad_nmea_sentence : std::exception {
    public:
        bad_nmea_sentence(std::string sentence_type, std::string sentence) {
            _sentence = sentence;
            _sentence_type = sentence_type;
        }

        const char *what() const throw (){
           return ("Bad nmea sentence of type " + _sentence_type + " : " + _sentence).c_str();
        }

        const std::string &sentence_type() const {
            return _sentence_type;
        }

        const std::string &sentence() const {
            return _sentence;
        }

    private:
        std::string _sentence_type;
        std::string _sentence;
    };
}

#endif
