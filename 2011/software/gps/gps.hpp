
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <list>



class gps
{
	public:
	struct GPSState
	{
		double time;
		double lat;
		double lon;
	};

	gps();
	~gps();

	bool open(char* port, size_t baud);
	void start();
	void stop();

	const GPSState& get_last_pos();

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
	size_t queuelen;
};
