#include "gps.hpp"
#include "nmea.hpp"

gps::gps() : running(false), gps_port(io_service)
{
	
}
gps::~gps()
{
	stop();
}

void gps::start()
{
	running = true;
	iothread = boost::thread(boost::bind(&gps::gps_comm, this));
}

void gps::stop()
{
	running = false;
	iothread.join();
}

const gps::GPSState& gps::get_last_pos()
{
	boost::mutex::scoped_lock lock(state_queue_mutex);
	return state_queue.back();
}

bool gps::open(char* port, size_t baud)
{
	gps_port.open(port);

	if(!gps_port.is_open())
	{
		return false;
	}
	
	gps_port.set_option(boost::asio::serial_port_base::baud_rate(baud));
	//gps_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
	gps_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
	gps_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

	return true;
}

void gps::gps_comm()
{
	while(running)
	{
		boost::asio::read_until(gps_port, comm_buffer, '\n');
		std::istream is(&comm_buffer);
		std::string line;
		std::getline(is, line);

		GPSState state;
		bool ret = parse_message(line, state);
	}
}

bool gps::parse_message(const std::string& line, GPSState& state)
{
	if(nmea::decodeGPGGA(line))
	{
		return true;
	}
	if(nmea::decodeGPRMC(line))
	{
		return true;
	}
	if(nmea::decodeGPRMT(line))
	{
		return true;
	}
	if(nmea::decodeGPGSA(line))
	{
		return true;
	}
	if(nmea::decodeGPGSV(line))
	{
		return true;
	}
	return false;
}


