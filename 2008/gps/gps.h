#include "thread.h"

// spec says 82, but that doesn't mean squat. Make it big
#define NMEA_MAX_SENTENCE_LEN 128

/* TODO:
	- Clean up names
	- Clean up gps_data_t
	- fix stuff from PLAYER_ERROR/PLAYER_WARN
		-- better error reporting
		-- die gracefully
	- get rid of pthread stuff (add it back later?)
	- add getters for gps_data
	- make it continue to try to connect
*/

/**
 * Device driver for the Garmin geko 201 handheld GPS unit.  Interacts with
 * the unit by speaking NMEA over a serial line.  As such, this driver may
 * work with other Garmin units, and (likely with some modification) other
 * NMEA-compliant GPS units.
 *
 * The driver may also attempt to read DGPS RTCM corrections from a
 * multi-cast network address, and forward these corrections to the
 * GPS unit.  The @ref util_dgps_server utility may be used to gather
 * and broadcast the DGPS RTCM corrections.
 *
 * NMEA and proprietary Garmin codes can be found at
 * http://home.mira.net/~gnb/gps/nmea.html
 *
 * Configuration options
 *
 * - port (string)
 *   - Default: "/dev/ttyS0"
 *   - Serial port where the GPS unit is connected
 * - baud (integer)
 *   - Default: 4800
 *   - Speed of serial connection to the GPS unit
 * - dgps_enable (integer)
 *   - Default: 1
 *   - Enable/disable listening for DGPS corrections via UDP multicast
 *     (use @ref util_dgps_server to send the corrections)
 * - dgps_group (string)
 *   - Default: "225.0.0.43"
 *   - Multicast group on which to listen for DGPS corrections</td></tr>
 * - dgps_port (integer)
 *   - Default: 7778
 *   - UDP port on which to listen for DGPS corrections
 */

//@todo: Review the units used here
typedef struct gps_data {
	/** GPS (UTC) time, in seconds and microseconds since the epoch. */
	uint32_t time_sec;
	/** GPS (UTC) time, in seconds and microseconds since the epoch. */
	uint32_t time_usec;
	/** Latitude in degrees / 1e7 (units are scaled such that the
      effective resolution is roughly 1cm).  Positive is north of
      equator, negative is south of equator. */
	int32_t latitude;
	/** Longitude in degrees / 1e7 (units are scaled such that the
      effective resolution is roughly 1cm).  Positive is east of prime
      meridian, negative is west of prime meridian. */
	int32_t longitude;
	/** Altitude, in millimeters.  Positive is above reference (e.g.,
      sea-level), and negative is below. */
	int32_t altitude;
	/** UTM WGS84 coordinates, easting [m] */
	double utm_e;
	/** UTM WGS84 coordinates, northing [m] */
	double utm_n;
	/** Quality of fix 0 = invalid, 1 = GPS fix, 2 = DGPS fix */
	uint32_t quality;
	/** Number of satellites in view. */
	uint32_t num_sats;
	/** Horizontal dilution of position (HDOP), times 10 */
	uint32_t hdop;
	/** Vertical dilution of position (VDOP), times 10 */
	uint32_t vdop;
	/** Horizonal error [m] */
	double err_horz;
	/** Vertical error [m] */
	double err_vert;
} gps_data_t;

//#define DEFAULT_GPS_PORT "/dev/ttyS0"
//#define DEFAULT_DGPS_GROUP "225.0.0.43"
//#define DEFAULT_DGPS_PORT 7778

class GPS : Thread {
public:
	// constructor
	GPS(const char* gps_serial_port="/dev/ttyS0", int gps_baud=7778);

	// accessors for GPS data
	uint32_t time_sec(void);
	uint32_t time_usec(void);
	int32_t latitude(void);
	int32_t longitude(void);
	int32_t altitude(void);
	double utm_e(void);
	double utm_n(void);
	uint32_t quality(void);
	uint32_t num_sats(void);
	uint32_t hdop(void);
	uint32_t vdop(void);
	double err_horz(void);
	double err_vert(void);

private:
	// Code that gets ran in the thread
	void Run();

	// Destructor
	~GPS();

	// string name of serial port to use
	const char* gps_serial_port;

	// GPS baud rate
	int gps_baud;

	// file descriptor for the gps unit
	int gps_fd;  

	// Enable DGPS corrections?
	int dgps_enable;
  
	// Port number for DGPS RTCM corrections
	const char *dgps_group;
	int dgps_port;

	// file descriptor for the DGPS UDP socket
	int dgps_fd;
	bool gps_fd_blocking;

	char nmea_buf[NMEA_MAX_SENTENCE_LEN+1];
	size_t nmea_buf_len;

	// Status
	int read_count;

	// Filtered GPS geodetic coords; for outlier rejection
	double filter_a, filter_thresh;
	double filter_lat, filter_lon;
	bool filter_good;

	// Current GPS data packet
	gps_data_t data;

	int SetupSerial();
	void ShutdownSerial();

	int SetupSocket();
	void ShutdownSocket();

	int ReadSentence(char* buf, size_t len);
	int WriteSentence(const char *buf, size_t len);
	int ReadSocket(char *buf, size_t len);

	int FillBuffer();
	int ParseSentence(const char* buf);
	int ParseGPGGA(const char *buf);
	int ParseGPRMC(const char *buf);
	int ParsePGRME(const char *buf);
	int ParseGPGST(const char *buf);
	char* GetNextField(char* field, size_t len, const char* ptr);

	// utility functions to convert geodetic to UTM position
	void UTM(double lat, double lon, double *x, double *y);
};

