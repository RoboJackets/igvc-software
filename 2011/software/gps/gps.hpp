#ifndef GPS
#define GPS

#include "gps_common.hpp"

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <list>



class gps
{
	public:
	gps();
	~gps();

	bool open(const char* port, size_t baud);
	void close();
	void start();
	void stop();

	bool get_last_state(GPSState& state);

	private:
	volatile bool running;

	boost::asio::streambuf comm_buffer;
	void gps_comm();
	void handle_serial_read(const boost::system::error_code& ec, size_t len, boost::asio::deadline_timer& timeout);
	void handle_serial_read_timer(const boost::system::error_code& ec);

	void reconnect();

	bool parse_message(const std::string& line, GPSState& state);

	boost::thread iothread;
	boost::asio::io_service io_service;
	boost::asio::serial_port gps_port;
	volatile bool m_connected;
	volatile bool readPending;
	std::string m_port;
	size_t m_baud;

	boost::mutex state_queue_mutex;
	std::list<GPSState> state_queue;
	size_t queue_len;

	const static size_t SERIAL_TIMOUT_SEC = 2;
};

#endif
