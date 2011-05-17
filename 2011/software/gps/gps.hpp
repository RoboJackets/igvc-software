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

	bool open(char* port, size_t baud);
	void start();
	void stop();

	bool get_last_pos(GPSState& state);

	private:
	bool running;

	boost::asio::streambuf comm_buffer;
	void gps_comm();

	bool parse_message(const std::string& line, GPSState& state);

	boost::thread iothread;
	boost::asio::io_service io_service;
	boost::asio::serial_port gps_port;

	boost::mutex state_queue_mutex;
	std::list<GPSState> state_queue;
	size_t queue_len;
};

#endif
