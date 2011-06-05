#ifndef GYRO_HPP
#define GYRO_HPP

#include "gps_common.hpp"

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <list>



class gyro
{
	public:
	gyro();
	~gyro();

	bool open(const char* port, size_t baud);
	void close();
	void start();
	void stop();

	bool get_last_state(gyroState& state);
	bool get_heading(double& heading);
	
	private:
	volatile bool running;

	boost::asio::streambuf comm_buffer;
	void gyro_comm();
	void handle_serial_read(const boost::system::error_code& ec, size_t len, boost::asio::deadline_timer& timeout);
	void handle_serial_read_timer(const boost::system::error_code& ec);

	void reconnect();

	bool parse_message(const std::string& line, gyroState& state);

	boost::thread iothread;
	boost::asio::io_service io_service;
	boost::asio::serial_port gyro_port;
	volatile bool m_connected;
	volatile bool readPending;
	std::string m_port;
	size_t m_baud;

	boost::mutex state_queue_mutex;
	std::list<gyroState> state_queue;
	size_t queue_len;

	const static size_t SERIAL_TIMOUT_SEC = 2;
};

#endif
